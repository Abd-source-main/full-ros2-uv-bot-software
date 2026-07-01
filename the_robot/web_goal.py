"""
 open http://localhost:8080 in a browser.

Pixel -> map conversion (image origin is top-left, map origin is bottom-left):
    world_x = origin_x + px_x * resolution
    world_y = origin_y + (height - px_y) * resolution
"""

import io
import json
import os
import subprocess
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import yaml
from PIL import Image


def _default_map():
    """Locate maps/hospital.yaml from the installed share dir or the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = get_package_share_directory('the_robot')
        cand = os.path.join(share, 'maps', 'hospital.yaml')
        if os.path.isfile(cand):
            return cand
    except Exception:
        pass
    # Fall back to the source tree: <pkg>/the_robot/web_goal.py -> <pkg>/maps/...
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(os.path.dirname(here), 'maps', 'hospital.yaml')


class MapData:
    """Loads the map image + metadata and converts pixels to map coordinates."""

    def __init__(self, yaml_path):
        with open(yaml_path) as f:
            meta = yaml.safe_load(f)
        self.resolution = float(meta['resolution'])
        self.origin = [float(v) for v in meta['origin']]
        img_path = meta['image']
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(yaml_path), img_path)
        self.image = Image.open(img_path).convert('L') # convert to grayscale if not already
        self.width, self.height = self.image.size
        # Pre-encode the map as PNG once (browsers can't show PGM).
        buf = io.BytesIO()
        self.image.save(buf, format='PNG')
        self.png = buf.getvalue()
        

    def pixel_to_world(self, px_x, px_y):
        x = self.origin[0] + px_x * self.resolution
        y = self.origin[1] + (self.height - px_y) * self.resolution
        return x, y

    def meta_json(self):
        return {
            'width': self.width,
            'height': self.height,
            'resolution': self.resolution,
            'origin': self.origin,
        }


def send_goal(x, y, yaw):
    """Invoke the nav2_goal node to publish a single goal. Returns (ok, output)."""
    cmd = [
        'ros2', 'run', 'the_robot', 'nav2_goal', '--ros-args',
        '-p', f'x:={x}', '-p', f'y:={y}', '-p', f'yaw:={yaw}',
    ]
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, timeout=30,
        )
    except FileNotFoundError:
        return False, "'ros2' not found. Source your workspace before launching web_goal."
    except subprocess.TimeoutExpired as exc:
        return False, f'nav2_goal timed out after 30s:\n{exc.stdout or ""}{exc.stderr or ""}'
    output = (proc.stdout or '') + (proc.stderr or '')
    ok = proc.returncode == 0 and 'Published Nav2 goal' in output
    return ok, output


INDEX_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Nav2 Goal Picker</title>
<style>
  :root { color-scheme: dark; }
  body { margin: 0; font-family: system-ui, sans-serif; background: #14161a; color: #e7e9ee; }
  header { padding: 12px 20px; background: #1d2127; border-bottom: 1px solid #2a2f37; }
  header h1 { margin: 0; font-size: 16px; font-weight: 600; }
  header p { margin: 4px 0 0; font-size: 12px; color: #9aa1ad; }
  main { display: flex; gap: 20px; padding: 20px; flex-wrap: wrap; }
  .mapwrap { background: #0e0f12; border: 1px solid #2a2f37; border-radius: 8px; padding: 8px; }
  canvas { display: block; cursor: crosshair; image-rendering: pixelated; border-radius: 4px; }
  aside { min-width: 260px; flex: 1; }
  .card { background: #1d2127; border: 1px solid #2a2f37; border-radius: 8px; padding: 16px; margin-bottom: 16px; }
  .row { display: flex; justify-content: space-between; padding: 4px 0; font-size: 14px; }
  .row span:first-child { color: #9aa1ad; }
  .coord { font-variant-numeric: tabular-nums; font-weight: 600; }
  button { width: 100%; padding: 10px; font-size: 14px; font-weight: 600; border: 0; border-radius: 6px;
           background: #3b82f6; color: white; cursor: pointer; }
  button:disabled { background: #394150; color: #79808d; cursor: not-allowed; }
  label { font-size: 13px; color: #9aa1ad; display: block; margin-bottom: 6px; }
  input[type=range] { width: 100%; }
  pre { background: #0e0f12; border: 1px solid #2a2f37; border-radius: 6px; padding: 10px;
        font-size: 12px; max-height: 220px; overflow: auto; white-space: pre-wrap; }
  .status { font-size: 13px; margin-top: 8px; min-height: 18px; }
  .ok { color: #4ade80; } .err { color: #f87171; } .pending { color: #fbbf24; }
</style>
</head>
<body>
<header>
  <h1>Nav2 Goal Picker</h1>
  <p>Click anywhere on the map to send the robot there.</p>
</header>
<main>
  <div class="mapwrap"><canvas id="map" width="500" height="500"></canvas></div>
  <aside>
    <div class="card">
      <div class="row"><span>Clicked pixel</span><span class="coord" id="px">—</span></div>
      <div class="row"><span>Map X (m)</span><span class="coord" id="wx">—</span></div>
      <div class="row"><span>Map Y (m)</span><span class="coord" id="wy">—</span></div>
      <div style="margin-top:12px">
        <label>Heading (yaw): <span id="yawlbl">0°</span></label>
        <input type="range" id="yaw" min="-180" max="180" value="0" step="5">
      </div>
      <div style="margin-top:14px">
        <button id="go" disabled>Send goal</button>
      </div>
      <div class="status" id="status"></div>
    </div>
    <div class="card">
      <label>nav2_goal output</label>
      <pre id="log">No goal sent yet.</pre>
    </div>
  </aside>
</main>
<script>
let meta = null, pick = null;
const cv = document.getElementById('map'), ctx = cv.getContext('2d');
const img = new Image();

async function init() {
  meta = await (await fetch('/meta')).json();
  cv.width = meta.width; cv.height = meta.height;
  img.onload = () => draw();
  img.src = '/map.png';
}

function draw() {
  ctx.drawImage(img, 0, 0, cv.width, cv.height);
  if (pick) {
    ctx.strokeStyle = '#3b82f6'; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(pick.px, pick.py, 7, 0, 2*Math.PI); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(pick.px-12, pick.py); ctx.lineTo(pick.px+12, pick.py);
    ctx.moveTo(pick.px, pick.py-12); ctx.lineTo(pick.px, pick.py+12); ctx.stroke();
  }
}

cv.addEventListener('click', (e) => {
  const r = cv.getBoundingClientRect();
  // Map CSS coords back to natural image pixels.
  const px = (e.clientX - r.left) * (cv.width / r.width);
  const py = (e.clientY - r.top) * (cv.height / r.height);
  const wx = meta.origin[0] + px * meta.resolution;
  const wy = meta.origin[1] + (meta.height - py) * meta.resolution;
  pick = { px, py, wx, wy };
  document.getElementById('px').textContent = `${px.toFixed(0)}, ${py.toFixed(0)}`;
  document.getElementById('wx').textContent = wx.toFixed(3);
  document.getElementById('wy').textContent = wy.toFixed(3);
  document.getElementById('go').disabled = false;
  draw();
});

const yaw = document.getElementById('yaw');
yaw.addEventListener('input', () => {
  document.getElementById('yawlbl').textContent = yaw.value + '°';
});

document.getElementById('go').addEventListener('click', async () => {
  if (!pick) return;
  const btn = document.getElementById('go'), st = document.getElementById('status');
  btn.disabled = true; st.className = 'status pending'; st.textContent = 'Sending goal…';
  const yawRad = (parseFloat(yaw.value) * Math.PI / 180);
  try {
    const res = await fetch('/goto', {
      method: 'POST', headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ x: pick.wx, y: pick.wy, yaw: yawRad }),
    });
    const data = await res.json();
    st.className = 'status ' + (data.ok ? 'ok' : 'err');
    st.textContent = data.ok ? `Goal sent: x=${pick.wx.toFixed(2)}, y=${pick.wy.toFixed(2)}`
                             : 'Failed — see output below.';
    document.getElementById('log').textContent = data.output || '(no output)';
  } catch (err) {
    st.className = 'status err'; st.textContent = 'Request error: ' + err;
  } finally {
    btn.disabled = false;
  }
});

init();
</script>
</body>
</html>
"""


def make_handler(mapdata):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):  # quieter console
            pass

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/' or self.path.startswith('/index'):
                self._send(200, 'text/html; charset=utf-8', INDEX_HTML.encode())
            elif self.path == '/map.png':
                self._send(200, 'image/png', mapdata.png)
            elif self.path == '/meta':
                self._send(200, 'application/json',
                           json.dumps(mapdata.meta_json()).encode())
            else:
                self._send(404, 'text/plain', b'not found')

        def do_POST(self):
            if self.path != '/goto':
                self._send(404, 'text/plain', b'not found')
                return
            length = int(self.headers.get('Content-Length', 0))
            try:
                req = json.loads(self.rfile.read(length) or b'{}')
                x, y = float(req['x']), float(req['y'])
                yaw = float(req.get('yaw', 0.0))
            except (ValueError, KeyError, TypeError) as exc:
                self._send(400, 'application/json',
                           json.dumps({'ok': False, 'output': f'bad request: {exc}'}).encode())
                return
            ok, output = send_goal(x, y, yaw)
            self._send(200, 'application/json',
                       json.dumps({'ok': ok, 'output': output}).encode())

    return Handler


def main(args=None):
    map_path = _default_map()
    host, port = '0.0.0.0', 8080

    mapdata = MapData(map_path)
    server = ThreadingHTTPServer((host, port), make_handler(mapdata))
    url = f'http://localhost:{port}'
    print(f'[web_goal] Map: {map_path} ({mapdata.width}x{mapdata.height}, '
          f'{mapdata.resolution} m/px, origin {mapdata.origin[:2]})')
    print(f'[web_goal] Serving on {url}  (Ctrl+C to stop)')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print('\n[web_goal] Shutting down.')
        server.shutdown()


if __name__ == '__main__':
    main()
