"""Web driving app for the_robot.

Open http://localhost:8081 in a browser to DRIVE the robot with on-screen
buttons or the W/A/S/D keys, while watching the SLAM map build up live.

Design
------
* An rclpy node publishes geometry_msgs/Twist on /cmd_vel at a fixed rate.
* The browser sends the desired {linear, angular} at ~10 Hz while a control is
  held, and a /stop when released. A server-side deadman watchdog zeroes the
  velocity if no command arrives within `cmd_timeout` seconds, so a dropped
  connection or a stuck key can never leave the robot driving.
* The node subscribes to /map (the slam_toolbox occupancy grid, TRANSIENT_LOCAL
  QoS) and renders it to a PNG on demand; the page refreshes it every second so
  you literally see the map grow as you drive.
"""

import io
import json
import math
import threading
import time
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
    """Publishes /cmd_vel from web commands and serves the live SLAM map."""

    def __init__(self):
        super().__init__('web_teleop')

        self.declare_parameter('max_linear', 0.22)     # m/s
        self.declare_parameter('max_angular', 0.5)     # rad/s
        self.declare_parameter('publish_rate', 20.0)   # Hz
        self.declare_parameter('cmd_timeout', 0.5)     # s deadman
        self.declare_parameter('port', 8081)

        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout').value)
        self.port = int(self.get_parameter('port').value)
        rate = float(self.get_parameter('publish_rate').value)

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd = 0.0
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
    def set_command(self, linear, angular):
        with self.lock:
            self.target_linear = max(-self.max_linear,
                                     min(self.max_linear, float(linear)))
            self.target_angular = max(-self.max_angular,
                                      min(self.max_angular, float(angular)))
            self.last_cmd = time.monotonic()

    def stop(self):
        with self.lock:
            self.target_linear = 0.0
            self.target_angular = 0.0
            self.last_cmd = time.monotonic()

    def _publish(self):
        with self.lock:
            # Deadman: if the browser went quiet, zero the velocity.
            if time.monotonic() - self.last_cmd > self.cmd_timeout:
                self.target_linear = 0.0
                self.target_angular = 0.0
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
  .joywrap { display: flex; justify-content: center; }
  .joy { position: relative; width: 220px; height: 220px; border-radius: 50%;
         background: radial-gradient(circle at 50% 50%, #232833 0%, #171a20 70%, #12151a 100%);
         border: 1px solid #2a2f37; touch-action: none; user-select: none; }
  .joy::before, .joy::after { content: ''; position: absolute; background: #2a2f37; }
  .joy::before { left: 50%; top: 10%; width: 1px; height: 80%; transform: translateX(-50%); }
  .joy::after { top: 50%; left: 10%; height: 1px; width: 80%; transform: translateY(-50%); }
  .knob { position: absolute; left: 50%; top: 50%; width: 84px; height: 84px; border-radius: 50%;
          transform: translate(-50%, -50%); background: #3b82f6; box-shadow: 0 4px 14px rgba(0,0,0,.5);
          cursor: grab; transition: transform .08s ease-out; }
  .knob.active { cursor: grabbing; transition: none; }
  .stopbtn { width: 100%; margin-top: 14px; padding: 10px; font-size: 14px; font-weight: 600; border: 0;
             border-radius: 6px; background: #ef4444; color: #fff; cursor: pointer; }
  .row { display: flex; justify-content: space-between; padding: 4px 0; font-size: 14px; }
  .row span:first-child { color: #9aa1ad; }
  .coord { font-variant-numeric: tabular-nums; font-weight: 600; }
  .hint { font-size: 12px; color: #9aa1ad; margin-top: 10px; line-height: 1.5; }
  .status { font-size: 13px; margin-top: 8px; min-height: 18px; color: #9aa1ad; }
</style>
</head>
<body>
<header>
  <h1>Drive &amp; Map</h1>
  <p>Drag the joystick (or use W / A / S / D) to drive. The map builds as you go &mdash; it is saved on exit.</p>
</header>
<main>
  <div class="mapwrap"><img id="map" alt="SLAM map (waiting for /map)…"></div>
  <aside>
    <div class="card">
      <div class="joywrap">
        <div class="joy" id="joy"><div class="knob" id="knob"></div></div>
      </div>
      <button class="stopbtn" id="stop">STOP</button>
      <div class="hint">Push forward/back for speed, left/right to turn &mdash; both at once for
        arcs. Release (or press STOP) and the robot halts within ~0.5&nbsp;s.</div>
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
const MAXLIN = __MAXLIN__, MAXANG = __MAXANG__;

// Command sources. The joystick contributes an analog (already-scaled) value;
// the keyboard contributes normalized [-1,1] axes. They are summed and clamped.
let joyActive = false, joyLin = 0, joyAng = 0;   // m/s, rad/s
const keys = new Set();                          // held W/A/S/D keys

function keyAxes() {
  let lin = 0, ang = 0;
  if (keys.has('w')) lin += 1; if (keys.has('s')) lin -= 1;
  if (keys.has('a')) ang += 1; if (keys.has('d')) ang -= 1;   // left = +angular (REP-103)
  return { lin: lin * MAXLIN, ang: ang * MAXANG };
}

function command() {
  const k = keyAxes();
  let lin = joyLin + k.lin, ang = joyAng + k.ang;
  lin = Math.max(-MAXLIN, Math.min(MAXLIN, lin));
  ang = Math.max(-MAXANG, Math.min(MAXANG, ang));
  const active = joyActive || keys.size > 0;
  return { lin, ang, active };
}

function updateReadout(c) {
  document.getElementById('lin').textContent = c.lin.toFixed(2) + ' m/s';
  document.getElementById('ang').textContent = c.ang.toFixed(2) + ' rad/s';
  document.getElementById('status').textContent = c.active ? 'driving' : 'idle';
}

async function send() {
  const c = command();
  updateReadout(c);
  try {
    await fetch(c.active ? '/cmd' : '/stop',
      { method: 'POST', headers: {'Content-Type':'application/json'},
        body: JSON.stringify({ linear: c.lin, angular: c.ang }) });
  } catch (e) { document.getElementById('status').textContent = 'connection lost'; }
}

// Re-send at 10 Hz while active to keep the server deadman satisfied. Individual
// input events update local state only; this loop (plus an immediate send on
// press/release) does the posting, so fast dragging never floods the server.
setInterval(() => { if (command().active) send(); }, 100);

// --- virtual joystick (pointer events cover mouse + touch) ---
const joy = document.getElementById('joy'), knob = document.getElementById('knob');
let joyId = null;
const travel = () => (joy.clientWidth - knob.clientWidth) / 2;   // max knob offset (px)

function moveKnob(dx, dy) {
  knob.style.transform = `translate(calc(-50% + ${dx}px), calc(-50% + ${dy}px))`;
}
function joyTo(e) {
  const r = joy.getBoundingClientRect();
  let dx = e.clientX - (r.left + r.width / 2);
  let dy = e.clientY - (r.top + r.height / 2);
  const max = travel(), dist = Math.hypot(dx, dy);
  if (dist > max && dist > 0) { dx = dx / dist * max; dy = dy / dist * max; }
  moveKnob(dx, dy);
  const nx = dx / max, ny = -dy / max;   // ny up = forward; nx right = right turn
  joyLin = ny * MAXLIN;
  joyAng = -nx * MAXANG;
  updateReadout(command());
}
joy.addEventListener('pointerdown', (e) => {
  e.preventDefault(); joyId = e.pointerId; joyActive = true;
  joy.setPointerCapture(joyId); knob.classList.add('active');
  joyTo(e); send();
});
joy.addEventListener('pointermove', (e) => { if (e.pointerId === joyId) joyTo(e); });
function joyEnd(e) {
  if (joyId === null || (e && e.pointerId !== joyId)) return;
  joyId = null; joyActive = false; joyLin = 0; joyAng = 0;
  knob.classList.remove('active'); moveKnob(0, 0); send();
}
joy.addEventListener('pointerup', joyEnd);
joy.addEventListener('pointercancel', joyEnd);

function fullStop() {
  joyId = null; joyActive = false; joyLin = 0; joyAng = 0;
  keys.clear(); knob.classList.remove('active'); moveKnob(0, 0); send();
}
document.getElementById('stop').addEventListener('click', fullStop);

// --- keyboard ---
const KEYSET = new Set(['w', 'a', 's', 'd']);
window.addEventListener('keydown', (e) => {
  const k = e.key.toLowerCase();
  if (k === ' ') { e.preventDefault(); fullStop(); return; }
  if (!KEYSET.has(k) || e.repeat) return;
  keys.add(k); send();
});
window.addEventListener('keyup', (e) => {
  const k = e.key.toLowerCase();
  if (KEYSET.has(k)) { keys.delete(k); send(); }
});
window.addEventListener('blur', fullStop);

// Live map refresh.
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

        def do_GET(self):
            if self.path == '/' or self.path.startswith('/index'):
                html = (INDEX_HTML
                        .replace('__MAXLIN__', repr(node.max_linear))
                        .replace('__MAXANG__', repr(node.max_angular)))
                self._send(200, 'text/html; charset=utf-8', html.encode())
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
            length = int(self.headers.get('Content-Length', 0))
            raw = self.rfile.read(length) if length else b'{}'
            if self.path == '/stop':
                node.stop()
                self._send(200, 'application/json', b'{"ok": true}')
                return
            if self.path != '/cmd':
                self._send(404, 'text/plain', b'not found')
                return
            try:
                req = json.loads(raw or b'{}')
                lin = float(req.get('linear', 0.0))
                ang = float(req.get('angular', 0.0))
                if not (math.isfinite(lin) and math.isfinite(ang)):
                    raise ValueError('non-finite command')
            except (ValueError, TypeError) as exc:
                self._send(400, 'application/json',
                           json.dumps({'ok': False, 'error': str(exc)}).encode())
                return
            node.set_command(lin, ang)
            self._send(200, 'application/json', b'{"ok": true}')

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
