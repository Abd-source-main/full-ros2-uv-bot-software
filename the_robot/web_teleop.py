"""Web driving app for the_robot.

Open http://localhost:8081 in a browser to DRIVE the robot with an on-screen
arrow pad (↑ ↓ ← →) or the W/A/S/D / arrow keys, while watching the SLAM map
build up live.

Design
------
Same "tap to accelerate" model as teleop_wasd: each press *increments* the
target speed toward its max, rather than hold-to-move. The current target is
republished continuously so the robot keeps moving after a single tap.

    ↑ / w : faster forward     ↓ / s : faster backward
    ← / a : turn left faster    → / d : turn right faster
    STOP / space / e           : reset speed to zero

* An rclpy node publishes geometry_msgs/Twist on /cmd_vel at a fixed rate,
  holding the accumulated target (the motor_driver watchdog still stops the
  robot if this node dies). Press STOP to halt.
* The node subscribes to /map (the slam_toolbox occupancy grid, TRANSIENT_LOCAL
  QoS) and renders it to a PNG on demand; the page refreshes it every second so
  you literally see the map grow as you drive.
"""

import io
import json
import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node

# Pillow is only needed to render the live SLAM map. On a bare robot (joystick
# driving, no SLAM) it may not be installed -- keep it optional so the driving
# server still comes up. Without it, map rendering is simply disabled.
try:
    from PIL import Image
except ImportError:
    Image = None
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

# Occupancy value (0..100, or -1 unknown) -> 8-bit grayscale for the PNG.
# 255 in the raw byte stream is the unknown cell (-1 as unsigned); map it to a
# mid-gray so it reads as "not seen yet".
def _occ_table():
    table = bytearray(256)
    for i in range(256):
        if i == 255:            # -1 unknown
            table[i] = 205
        elif i <= 100:          # 0 free ... 100 occupied
            table[i] = int(round(254 * (1.0 - i / 100.0)))
        else:                   # out-of-range -> treat as unknown
            table[i] = 205
    return bytes(table)


OCC_TABLE = _occ_table()


class WebTeleop(Node):
    """Publishes /cmd_vel from web arrow-pad taps and serves the live SLAM map."""

    def __init__(self):
        super().__init__('web_teleop')

        # Match teleop_wasd: these are the *max* reachable speeds, and each tap
        # adds one step toward them.
        self.declare_parameter('max_linear', 0.12)     # m/s
        self.declare_parameter('max_angular', 0.4)     # rad/s
        self.declare_parameter('linear_step', 0.03)    # m/s added per ↑/↓ tap
        self.declare_parameter('angular_step', 0.1)    # rad/s added per ←/→ tap
        self.declare_parameter('publish_rate', 20.0)   # Hz to republish target
        self.declare_parameter('port', 8081)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.linear_step = float(self.get_parameter('linear_step').value)
        self.angular_step = float(self.get_parameter('angular_step').value)
        self.port = int(self.get_parameter('port').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.lock = threading.Lock()

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(1.0 / rate, self._publish)

        # slam_toolbox latches /map with TRANSIENT_LOCAL + RELIABLE; match it so
        # a late-joining subscriber still receives the last map.
        map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.latest_map = None
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)

    # --- command handling -------------------------------------------------
    def bump(self, d_linear, d_angular):
        """Adjust the target velocity by one step, clamped to the max.

        d_linear / d_angular are direction multipliers (+1, 0 or -1); returns
        the new (linear, angular) target so the page can update its readout.
        """
        with self.lock:
            self.target_linear += d_linear * self.linear_step
            self.target_angular += d_angular * self.angular_step
            self.target_linear = max(-self.max_linear,
                                     min(self.max_linear, self.target_linear))
            self.target_angular = max(-self.max_angular,
                                      min(self.max_angular, self.target_angular))
            return self.target_linear, self.target_angular

    def stop(self):
        """Reset the accumulated target to zero (full stop)."""
        with self.lock:
            self.target_linear = 0.0
            self.target_angular = 0.0
            return 0.0, 0.0

    def _publish(self):
        with self.lock:
            lin, ang = self.target_linear, self.target_angular
        twist = Twist()
        twist.linear.x = lin
        twist.angular.z = ang
        self.pub.publish(twist)

    # --- map handling -----------------------------------------------------
    def _on_map(self, msg):
        self.latest_map = msg

    def map_png(self):
        """Render the latest OccupancyGrid to PNG bytes, or None if no map yet."""
        if Image is None:          # Pillow not installed -> map rendering disabled
            return None
        msg = self.latest_map
        if msg is None:
            return None
        w, h = msg.info.width, msg.info.height
        if w == 0 or h == 0:
            return None
        raw = bytes((v & 0xFF) for v in msg.data)
        gray = raw.translate(OCC_TABLE)
        img = Image.frombytes('L', (w, h), gray)
        # ROS grids are row-major from the bottom-left; flip to image top-left.
        img = img.transpose(Image.FLIP_TOP_BOTTOM)
        buf = io.BytesIO()
        img.save(buf, format='PNG')
        return buf.getvalue()

    def map_meta(self):
        msg = self.latest_map
        if msg is None:
            return {}
        return {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
        }


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Drive & Map</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; font-family: system-ui, sans-serif; background: #14161a; color: #e7e9ee; }
  header { padding: 12px 20px; background: #1d2127; border-bottom: 1px solid #2a2f37; }
  header h1 { margin: 0; font-size: 16px; font-weight: 600; }
  header p { margin: 4px 0 0; font-size: 12px; color: #9aa1ad; }
  main { display: flex; gap: 20px; padding: 20px; flex-wrap: wrap; align-items: flex-start; }
  .mapwrap { background: #0e0f12; border: 1px solid #2a2f37; border-radius: 8px; padding: 8px; }
  #map { display: block; image-rendering: pixelated; border-radius: 4px; width: 500px; height: auto;
         background: #0e0f12; }
  aside { min-width: 240px; }
  .card { background: #1d2127; border: 1px solid #2a2f37; border-radius: 8px; padding: 16px; margin-bottom: 16px; }
  .padwrap { display: flex; justify-content: center; }
  .dpad { display: grid; grid-template-columns: repeat(3, 72px); grid-template-rows: repeat(3, 72px);
          gap: 8px; }
  .dpad button { font-size: 26px; line-height: 1; border: 1px solid #2a2f37; border-radius: 10px;
          background: #232833; color: #e7e9ee; cursor: pointer; user-select: none;
          touch-action: manipulation; transition: background .06s; }
  .dpad button:hover { background: #2b313c; }
  .dpad button:active { background: #3b82f6; }
  .dpad .up    { grid-column: 2; grid-row: 1; }
  .dpad .left  { grid-column: 1; grid-row: 2; }
  .dpad .stop  { grid-column: 2; grid-row: 2; background: #ef4444; color: #fff; font-size: 14px;
                 font-weight: 700; }
  .dpad .stop:hover { background: #dc2626; }
  .dpad .stop:active { background: #b91c1c; }
  .dpad .right { grid-column: 3; grid-row: 2; }
  .dpad .down  { grid-column: 2; grid-row: 3; }
  .row { display: flex; justify-content: space-between; padding: 4px 0; font-size: 14px; }
  .row span:first-child { color: #9aa1ad; }
  .coord { font-variant-numeric: tabular-nums; font-weight: 600; }
  .hint { font-size: 12px; color: #9aa1ad; margin-top: 14px; line-height: 1.5; }
  .status { font-size: 13px; margin-top: 8px; min-height: 18px; color: #9aa1ad; }
</style>
</head>
<body>
<header>
  <h1>Drive &amp; Map</h1>
  <p>Tap an arrow (or press W/A/S/D or the arrow keys) to speed up in that direction &mdash; tap again
     to go faster. The map builds as you go &mdash; it is saved on exit.</p>
</header>
<main>
  <div class="mapwrap"><img id="map" alt="SLAM map (waiting for /map)…"></div>
  <aside>
    <div class="card">
      <div class="padwrap">
        <div class="dpad">
          <button class="up"    id="up"    aria-label="forward">&#8593;</button>
          <button class="left"  id="left"  aria-label="turn left">&#8592;</button>
          <button class="stop"  id="stop"  aria-label="stop">STOP</button>
          <button class="right" id="right" aria-label="turn right">&#8594;</button>
          <button class="down"  id="down"  aria-label="backward">&#8595;</button>
        </div>
      </div>
      <div class="hint">Each tap adds a step of speed and the robot keeps going on its own.
        Press STOP (or space / e) to reset to a full stop.</div>
      <div class="status" id="status">idle</div>
    </div>
    <div class="card">
      <div class="row"><span>linear</span><span class="coord" id="lin">0.00 m/s</span></div>
      <div class="row"><span>angular</span><span class="coord" id="ang">0.00 rad/s</span></div>
      <div class="row"><span>map size</span><span class="coord" id="msize">—</span></div>
    </div>
  </aside>
</main>
<script>
function updateReadout(lin, ang) {
  document.getElementById('lin').textContent = lin.toFixed(2) + ' m/s';
  document.getElementById('ang').textContent = ang.toFixed(2) + ' rad/s';
  const moving = Math.abs(lin) > 1e-6 || Math.abs(ang) > 1e-6;
  document.getElementById('status').textContent = moving ? 'driving' : 'idle';
}

async function bump(dLin, dAng) {
  try {
    const r = await fetch('/bump', {
      method: 'POST', headers: {'Content-Type':'application/json'},
      body: JSON.stringify({ linear: dLin, angular: dAng }) });
    const j = await r.json();
    updateReadout(j.linear, j.angular);
  } catch (e) { document.getElementById('status').textContent = 'connection lost'; }
}

async function stop() {
  try {
    const r = await fetch('/stop', { method: 'POST' });
    const j = await r.json();
    updateReadout(j.linear, j.angular);
  } catch (e) { document.getElementById('status').textContent = 'connection lost'; }
}

// --- arrow pad buttons (left = +angular per REP-103) ---
document.getElementById('up').addEventListener('click',    () => bump(+1, 0));
document.getElementById('down').addEventListener('click',  () => bump(-1, 0));
document.getElementById('left').addEventListener('click',  () => bump(0, +1));
document.getElementById('right').addEventListener('click', () => bump(0, -1));
document.getElementById('stop').addEventListener('click', stop);

// --- keyboard (W/A/S/D + arrow keys, aliased) ---
const KEYMAP = {
  'w': [+1, 0], 'arrowup':    [+1, 0],
  's': [-1, 0], 'arrowdown':  [-1, 0],
  'a': [0, +1], 'arrowleft':  [0, +1],
  'd': [0, -1], 'arrowright': [0, -1],
};
window.addEventListener('keydown', (e) => {
  const k = e.key.toLowerCase();
  if (k === ' ' || k === 'e') { e.preventDefault(); stop(); return; }
  const step = KEYMAP[k];
  if (!step) return;
  e.preventDefault();       // stop arrow keys from scrolling the page
  if (e.repeat) return;     // one bump per deliberate press
  bump(step[0], step[1]);
});
// Safety: halt if the window loses focus (tab switch, minimise, etc.).
window.addEventListener('blur', stop);

// --- live map refresh ---
const mapImg = document.getElementById('map');
async function refreshMap() {
  try {
    const meta = await (await fetch('/map_meta')).json();
    if (meta.width) {
      document.getElementById('msize').textContent =
        meta.width + '×' + meta.height + ' @ ' + meta.resolution.toFixed(3) + ' m/px';
      mapImg.src = '/map.png?t=' + Date.now();
    }
  } catch (e) {}
}
setInterval(refreshMap, 1000);
refreshMap();
</script>
</body>
</html>
"""


def make_handler(node):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _send_targets(self, lin, ang):
            self._send(200, 'application/json',
                       json.dumps({'linear': lin, 'angular': ang}).encode())

        def do_GET(self):
            if self.path == '/' or self.path.startswith('/index'):
                self._send(200, 'text/html; charset=utf-8', INDEX_HTML.encode())
            elif self.path.startswith('/map.png'):
                png = node.map_png()
                if png is None:
                    self._send(503, 'text/plain', b'no map yet')
                else:
                    self._send(200, 'image/png', png)
            elif self.path == '/map_meta':
                self._send(200, 'application/json',
                           json.dumps(node.map_meta()).encode())
            else:
                self._send(404, 'text/plain', b'not found')

        def do_POST(self):
            if self.path == '/stop':
                lin, ang = node.stop()
                self._send_targets(lin, ang)
                return
            if self.path != '/bump':
                self._send(404, 'text/plain', b'not found')
                return
            length = int(self.headers.get('Content-Length', 0))
            raw = self.rfile.read(length) if length else b'{}'
            try:
                req = json.loads(raw or b'{}')
                d_lin = float(req.get('linear', 0.0))
                d_ang = float(req.get('angular', 0.0))
                if not (math.isfinite(d_lin) and math.isfinite(d_ang)):
                    raise ValueError('non-finite step')
            except (ValueError, TypeError) as exc:
                self._send(400, 'application/json',
                           json.dumps({'ok': False, 'error': str(exc)}).encode())
                return
            lin, ang = node.bump(d_lin, d_ang)
            self._send_targets(lin, ang)

    return Handler


def main(args=None):
    rclpy.init(args=args)
    node = WebTeleop()

    server = ThreadingHTTPServer(('0.0.0.0', node.port), make_handler(node))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(
        f'Drive & Map web app on http://localhost:{node.port}  (Ctrl+C to stop)')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.stop()
        node._publish()  # deliver a final zero velocity
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
