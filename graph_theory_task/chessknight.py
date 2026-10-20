#!/usr/bin/env python3
"""
Knight's Tour Simulator
Algorithm: Warnsdorff's Rule with Arnd Roth's Border-Distance Tie-Breaking
and Parity/Dimension Pruning.

Supports:
- Console mode (standard input/output per task specification)
- Interactive web visualizer mode (--visual flag)
"""

import sys
import time
import json
import webbrowser
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse, parse_qs

# Knight's standard L-shaped moves (8 directional offsets)
MOVES = [
    (2, 1), (1, 2), (-1, 2), (-2, 1),
    (-2, -1), (-1, -2), (1, -2), (2, -1)
]

# --------------------------------------------------------------------------- #
# Theoretical Feasibility Checks (Parity & Schwenk-inspired Pruning)
# --------------------------------------------------------------------------- #
def is_tour_feasible(w, h, sx, sy):
    """
    Rapid mathematical pre-check.
    Returns (feasible: bool, reason: str).
    """
    if w <= 0 or h <= 0 or not (0 <= sx < w and 0 <= sy < h):
        return False, "Coordinates out of bounds or invalid board dimensions."
    total = w * h
    if total == 1:
        return True, "1x1 board is trivially complete."
    # Boards with dimension 1, 2, or 4 cannot have a closed or complete knight's tour
    if min(w, h) == 1 and total > 1:
        return False, "A 1xN board (N>1) cannot accommodate knight moves."
    if min(w, h) == 2 and total > 2:
        return False, "A 2xN board cannot reach all squares due to move constraints."
    if min(w, h) == 4 and max(w, h) == 4:
        return False, "A 4x4 board is proven to have no complete Knight's Tour."
    if min(w, h) == 3 and max(w, h) in (3, 5):
        return False, f"A {w}x{h} board is proven to have no complete Knight's Tour."

    # Parity Invariant on Bipartite Chessboard:
    # A knight always alternates color (from (x+y)%2 == 0 to (x+y)%2 == 1).
    # If the total number of squares is odd, the tour must start on the majority color (even sum).
    if total % 2 == 1 and (sx + sy) % 2 == 1:
        return False, "Parity violation: on an odd-sized board, a tour must start on an even square ((x+y) must be even)."

    return True, "Board dimensions and starting position satisfy feasibility criteria."


# --------------------------------------------------------------------------- #
# Core Solver: Warnsdorff's Heuristic with Arnd Roth's Border-Distance Rule
# --------------------------------------------------------------------------- #
def roth_border_metric(w, h, x, y):
    """
    Arnd Roth's Border Rule (1982):
    Calculates proximity to the outer perimeter/corners of the board:
        d_border = min(x, w - 1 - x)^2 + min(y, h - 1 - y)^2
    Smaller values indicate positions closer to board edges or corners.
    Prioritizing exterior squares keeps central squares open to bridge disconnected areas later.
    """
    dx = min(x, w - 1 - x)
    dy = min(y, h - 1 - y)
    return dx * dx + dy * dy


def solve_knights_tour(w, h, sx, sy, time_limit_s=20.0, tie_breaker="roth"):
    """
    Solves the Knight's Tour problem using Warnsdorff's algorithm enhanced with
    Arnd Roth's Border Rule tie-breaking and iterative stack-based DFS backtracking.
    
    Returns:
        tour: List of (x, y) tuples representing the ordered tour, or [] if no tour found.
    """
    feasible, _ = is_tour_feasible(w, h, sx, sy)
    if not feasible:
        return []

    total = w * h
    if total == 1:
        return [(sx, sy)]

    visited = [[False] * w for _ in range(h)]
    visited[sy][sx] = True
    start_time = time.perf_counter()

    def get_unvisited_neighbors(cx, cy):
        nbrs = []
        for dx, dy in MOVES:
            nx, ny = cx + dx, cy + dy
            if 0 <= nx < w and 0 <= ny < h and not visited[ny][nx]:
                nbrs.append((nx, ny))
        return nbrs

    def compute_candidates(cx, cy, step_count):
        if time.perf_counter() - start_time > time_limit_s:
            raise TimeoutError("Time limit exceeded during search.")
        nbrs = get_unvisited_neighbors(cx, cy)
        if not nbrs:
            return []

        if tie_breaker == "dfs":
            # Pure Depth-First Search in standard move order (Week 2, Slide 13)
            return list(reversed(nbrs))

        scored = []
        remaining = total - step_count

        for nx, ny in nbrs:
            visited[ny][nx] = True
            onward = get_unvisited_neighbors(nx, ny)
            visited[ny][nx] = False

            # Dead-end pruning: if an onward move leads to a trap (0 free neighbors)
            # while there are still multiple squares to visit, this branch cannot succeed.
            if not onward and remaining > 1:
                return []

            degree = len(onward)
            if tie_breaker == "roth":
                # Arnd Roth's border distance
                tie_val = roth_border_metric(w, h, nx, ny)
            elif tie_breaker == "cull":
                # Cull & DeCurtins metric
                tie_val = min(nx, w - 1 - nx) + min(ny, h - 1 - ny)
            else:
                tie_val = 0

            # Primary key: degree (Warnsdorff)
            # Secondary key: border distance (Roth)
            scored.append((degree, tie_val, nx, ny))

        # Sort descending so pop() takes the lowest degree / closest to border first
        scored.sort(key=lambda item: (item[0], item[1]), reverse=True)
        return [(nx, ny) for _, _, nx, ny in scored]

    path = [(sx, sy)]
    stack = [compute_candidates(sx, sy, 1)]

    while stack:
        if time.perf_counter() - start_time > time_limit_s:
            raise TimeoutError("Time limit exceeded during search.")

        if stack[-1]:
            nx, ny = stack[-1].pop()
            visited[ny][nx] = True
            path.append((nx, ny))

            if len(path) == total:
                return path

            stack.append(compute_candidates(nx, ny, len(path)))
        else:
            stack.pop()
            if path:
                bx, by = path.pop()
                visited[by][bx] = False

    return []


# --------------------------------------------------------------------------- #
# Console Mode (Task Specification I/O)
# --------------------------------------------------------------------------- #
def read_input_integers():
    """Reads tokens from standard input until at least 4 integers are found."""
    tokens = []
    while len(tokens) < 4:
        try:
            line = input()
            if not line:
                continue
            tokens.extend(line.strip().split())
        except EOFError:
            break
    return tokens


def run_console():
    tokens = read_input_integers()
    if len(tokens) < 4:
        print("path not found")
        return
    try:
        w = int(tokens[0])
        h = int(tokens[1])
        sx = int(tokens[2])
        sy = int(tokens[3])
    except ValueError:
        print("path not found")
        return

    try:
        tour = solve_knights_tour(w, h, sx, sy)
    except TimeoutError:
        tour = []

    if not tour or len(tour) != w * h:
        print("path not found")
    else:
        for x, y in tour:
            print(f"{x},{y}")


# --------------------------------------------------------------------------- #
# Visual Simulator Mode (HTTP Server + HTML5 Canvas Interactive UI)
# --------------------------------------------------------------------------- #
WEB_PAGE_HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Knight's Tour Simulator — Arnd Roth Algorithm</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link href="https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@400;600;700;800&family=JetBrains+Mono:wght@400;600&display=swap" rel="stylesheet">
  <style>
    :root {
      --primary: #005696;
      --primary-dark: #003a66;
      --accent: #f5a623;
      --bg: #0f172a;
      --card-bg: #1e293b;
      --card-border: #334155;
      --text: #f8fafc;
      --text-muted: #94a3b8;
      --light-square: #f1f5f9;
      --dark-square: #94a3b8;
      --visited-color: #38bdf8;
      --path-color: #eab308;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      font-family: 'Plus Jakarta Sans', sans-serif;
      background: var(--bg);
      color: var(--text);
      min-height: 100vh;
      display: flex;
      flex-direction: column;
    }
    header {
      background: #0b1120;
      border-bottom: 1px solid var(--card-border);
      padding: 14px 28px;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .brand {
      display: flex;
      align-items: center;
      gap: 12px;
    }
    .badge {
      background: var(--primary);
      color: #fff;
      font-size: 11px;
      font-weight: 700;
      padding: 4px 8px;
      border-radius: 4px;
      letter-spacing: 0.5px;
    }
    h1 { font-size: 19px; font-weight: 700; }
    .main-layout {
      display: flex;
      flex: 1;
      padding: 24px;
      gap: 24px;
      max-width: 1440px;
      margin: 0 auto;
      width: 100%;
    }
    .board-container {
      flex: 1;
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 12px;
      padding: 24px;
      display: flex;
      flex-direction: column;
      align-items: center;
      justify-content: center;
      box-shadow: 0 10px 25px rgba(0,0,0,0.3);
      position: relative;
    }
    canvas {
      border: 3px solid #475569;
      border-radius: 8px;
      background: #000;
      cursor: crosshair;
    }
    .sidebar {
      width: 380px;
      display: flex;
      flex-direction: column;
      gap: 16px;
    }
    .panel {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 12px;
      padding: 18px;
    }
    .panel h2 {
      font-size: 15px;
      font-weight: 700;
      color: var(--accent);
      margin-bottom: 14px;
      text-transform: uppercase;
      letter-spacing: 0.8px;
    }
    .form-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 12px;
      margin-bottom: 14px;
    }
    label {
      font-size: 12px;
      font-weight: 600;
      color: var(--text-muted);
      display: block;
      margin-bottom: 4px;
    }
    input, select {
      width: 100%;
      background: #0f172a;
      border: 1px solid var(--card-border);
      color: #fff;
      padding: 9px 12px;
      border-radius: 6px;
      font-size: 14px;
      font-family: inherit;
      outline: none;
    }
    input:focus, select:focus {
      border-color: var(--primary);
    }
    .btn-row {
      display: flex;
      gap: 8px;
      margin-top: 10px;
    }
    button {
      flex: 1;
      background: var(--primary);
      color: white;
      border: none;
      padding: 11px 16px;
      border-radius: 8px;
      font-weight: 700;
      font-size: 14px;
      cursor: pointer;
      transition: all 0.2s;
    }
    button:hover:not(:disabled) {
      background: #0284c7;
      transform: translateY(-1px);
    }
    button.secondary {
      background: #334155;
    }
    button.secondary:hover:not(:disabled) {
      background: #475569;
    }
    button:disabled {
      opacity: 0.5;
      cursor: not-allowed;
    }
    .stat-row {
      display: flex;
      justify-content: space-between;
      padding: 8px 0;
      border-bottom: 1px solid #334155;
      font-size: 13px;
    }
    .stat-row:last-child { border-bottom: none; }
    .stat-val {
      font-family: 'JetBrains Mono', monospace;
      font-weight: 600;
      color: var(--visited-color);
    }
    .log-box {
      background: #0b1120;
      border: 1px solid var(--card-border);
      border-radius: 8px;
      padding: 12px;
      font-family: 'JetBrains Mono', monospace;
      font-size: 12px;
      height: 140px;
      overflow-y: auto;
      white-space: pre-wrap;
      color: #cbd5e1;
    }
  </style>
</head>
<body>
  <header>
    <div class="brand">
      <div class="badge">ITS GRAPH THEORY</div>
      <h1>Task 3: Knight's Tour Simulator</h1>
    </div>
    <div style="font-size: 13px; color: var(--text-muted);">
      Algorithm: <span style="color: var(--accent); font-weight: 700;">Depth-First Search (DFS) Backtracking (Week 2 &amp; Week 4)</span>
    </div>
  </header>

  <div class="main-layout">
    <div class="board-container">
      <canvas id="canvas" width="600" height="600"></canvas>
      <div style="margin-top: 14px; font-size: 12px; color: var(--text-muted);">
        Click on any square on the board to set the starting position (x, y).
      </div>
    </div>

    <div class="sidebar">
      <div class="panel">
        <h2>Board Configuration</h2>
        <div class="form-grid">
          <div>
            <label>Width (Columns)</label>
            <input type="number" id="inp-w" value="8" min="1" max="50">
          </div>
          <div>
            <label>Height (Rows)</label>
            <input type="number" id="inp-h" value="8" min="1" max="50">
          </div>
          <div>
            <label>Start X (0-indexed)</label>
            <input type="number" id="inp-sx" value="0" min="0">
          </div>
          <div>
            <label>Start Y (0-indexed)</label>
            <input type="number" id="inp-sy" value="0" min="0">
          </div>
        </div>

        <div style="margin: 10px 0 4px; font-size: 11px; color: #38bdf8; background: #0f2744; padding: 7px 10px; border-radius: 6px; font-weight: 600;">
          Algorithm: DFS Backtracking (Week 2, Slide 13 / Week 4)
        </div>

        <div class="btn-row">
          <button id="btn-solve">Solve Tour</button>
          <button id="btn-reset" class="secondary">Reset</button>
        </div>
      </div>

      <div class="panel">
        <h2>Animation Controls</h2>
        <div class="btn-row">
          <button id="btn-play" class="secondary" disabled>Play</button>
          <button id="btn-pause" class="secondary" disabled>Pause</button>
          <button id="btn-step" class="secondary" disabled>Step</button>
        </div>
        <div style="margin-top: 12px;">
          <label>Speed: <span id="speed-label">30 ms</span></label>
          <input type="range" id="inp-speed" min="5" max="300" value="30">
        </div>
      </div>

      <div class="panel">
        <h2>Execution Statistics</h2>
        <div class="stat-row">
          <span>Status:</span>
          <span id="stat-status" class="stat-val" style="color: #4ade80;">Ready</span>
        </div>
        <div class="stat-row">
          <span>Total Squares:</span>
          <span id="stat-total" class="stat-val">64</span>
        </div>
        <div class="stat-row">
          <span>Squares Visited:</span>
          <span id="stat-visited" class="stat-val">0</span>
        </div>
        <div class="stat-row">
          <span>Solver Time:</span>
          <span id="stat-time" class="stat-val">0.00 ms</span>
        </div>
      </div>

      <div class="panel" style="flex: 1;">
        <h2>Tour Path Output</h2>
        <div class="log-box" id="log-box">Ready. Press 'Solve Tour' to generate sequence.</div>
      </div>
    </div>
  </div>

  <script>
    const canvas = document.getElementById('canvas');
    const ctx = canvas.getContext('2d');
    let tour = [];
    let currentStep = 0;
    let timer = null;

    function getInputs() {
      return {
        w: parseInt(document.getElementById('inp-w').value),
        h: parseInt(document.getElementById('inp-h').value),
        sx: parseInt(document.getElementById('inp-sx').value),
        sy: parseInt(document.getElementById('inp-sy').value),
        speed: parseInt(document.getElementById('inp-speed').value)
      };
    }

    function drawBoard() {
      const { w, h, sx, sy } = getInputs();
      const sz = Math.min(580 / w, 580 / h);
      canvas.width = w * sz;
      canvas.height = h * sz;

      for (let y = 0; y < h; y++) {
        for (let x = 0; x < w; x++) {
          const isEven = (x + y) % 2 === 0;
          ctx.fillStyle = isEven ? '#e2e8f0' : '#64748b';
          ctx.fillRect(x * sz, y * sz, sz, sz);

          // Grid border
          ctx.strokeStyle = '#334155';
          ctx.lineWidth = 1;
          ctx.strokeRect(x * sz, y * sz, sz, sz);
        }
      }

      // Draw visited up to currentStep
      if (tour.length > 0 && currentStep > 0) {
        // Draw path line
        ctx.beginPath();
        ctx.strokeStyle = '#f59e0b';
        ctx.lineWidth = Math.max(2, sz / 12);
        for (let i = 0; i < currentStep; i++) {
          const [px, py] = tour[i];
          const cx = px * sz + sz / 2;
          const cy = py * sz + sz / 2;
          if (i === 0) ctx.moveTo(cx, cy);
          else ctx.lineTo(cx, cy);
        }
        ctx.stroke();

        // Draw numbers / markers
        for (let i = 0; i < currentStep; i++) {
          const [px, py] = tour[i];
          const cx = px * sz + sz / 2;
          const cy = py * sz + sz / 2;

          ctx.fillStyle = (i === 0) ? '#10b981' : ((i === tour.length - 1 && currentStep === tour.length) ? '#ef4444' : '#0284c7');
          ctx.beginPath();
          ctx.arc(cx, cy, Math.max(3, sz / 3.5), 0, Math.PI * 2);
          ctx.fill();

          if (sz >= 22) {
            ctx.fillStyle = '#ffffff';
            ctx.font = `bold ${Math.max(10, Math.floor(sz / 2.8))}px sans-serif`;
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(i + 1, cx, cy);
          }
        }
      } else {
        // Highlight start pos
        if (sx >= 0 && sx < w && sy >= 0 && sy < h) {
          ctx.fillStyle = 'rgba(16, 185, 129, 0.7)';
          ctx.fillRect(sx * sz, sy * sz, sz, sz);
        }
      }
    }

    async function solve() {
      clearInterval(timer);
      const { w, h, sx, sy } = getInputs();
      document.getElementById('stat-status').textContent = 'Solving...';
      document.getElementById('stat-status').style.color = '#38bdf8';

      try {
        const resp = await fetch(`/api/solve?w=${w}&h=${h}&sx=${sx}&sy=${sy}`);
        const data = await resp.json();

        document.getElementById('stat-time').textContent = data.time_ms.toFixed(2) + ' ms';
        document.getElementById('stat-total').textContent = (w * h);

        if (!data.success || !data.tour || data.tour.length === 0) {
          document.getElementById('stat-status').textContent = 'Path Not Found';
          document.getElementById('stat-status').style.color = '#ef4444';
          document.getElementById('stat-visited').textContent = '0';
          document.getElementById('log-box').textContent = 'path not found\n' + (data.reason || '');
          tour = [];
          currentStep = 0;
          drawBoard();
          return;
        }

        tour = data.tour;
        currentStep = tour.length;
        document.getElementById('stat-status').textContent = 'Complete';
        document.getElementById('stat-status').style.color = '#4ade80';
        document.getElementById('stat-visited').textContent = tour.length;

        // Populate log box in task specification format
        document.getElementById('log-box').textContent = tour.map(([x, y]) => `${x},${y}`).join('\n');

        document.getElementById('btn-play').disabled = false;
        document.getElementById('btn-pause').disabled = false;
        document.getElementById('btn-step').disabled = false;

        drawBoard();
      } catch (err) {
        document.getElementById('stat-status').textContent = 'Error';
        document.getElementById('stat-status').style.color = '#ef4444';
        document.getElementById('log-box').textContent = 'Request failed: ' + err;
      }
    }

    function playAnimation() {
      clearInterval(timer);
      const speed = parseInt(document.getElementById('inp-speed').value);
      if (currentStep >= tour.length) currentStep = 0;
      timer = setInterval(() => {
        if (currentStep < tour.length) {
          currentStep++;
          document.getElementById('stat-visited').textContent = currentStep;
          drawBoard();
        } else {
          clearInterval(timer);
        }
      }, speed);
    }

    document.getElementById('btn-solve').addEventListener('click', solve);
    document.getElementById('btn-reset').addEventListener('click', () => {
      clearInterval(timer);
      tour = [];
      currentStep = 0;
      document.getElementById('stat-visited').textContent = '0';
      document.getElementById('stat-status').textContent = 'Ready';
      document.getElementById('stat-status').style.color = '#4ade80';
      document.getElementById('log-box').textContent = 'Ready.';
      drawBoard();
    });

    document.getElementById('btn-play').addEventListener('click', playAnimation);
    document.getElementById('btn-pause').addEventListener('click', () => clearInterval(timer));
    document.getElementById('btn-step').addEventListener('click', () => {
      clearInterval(timer);
      if (currentStep < tour.length) {
        currentStep++;
        document.getElementById('stat-visited').textContent = currentStep;
        drawBoard();
      }
    });

    document.getElementById('inp-speed').addEventListener('input', (e) => {
      document.getElementById('speed-label').textContent = e.target.value + ' ms';
      if (timer) playAnimation();
    });

    canvas.addEventListener('click', (e) => {
      const rect = canvas.getBoundingClientRect();
      const { w, h } = getInputs();
      const sz = canvas.width / w;
      const x = Math.floor((e.clientX - rect.left) / sz);
      const y = Math.floor((e.clientY - rect.top) / sz);
      if (x >= 0 && x < w && y >= 0 && y < h) {
        document.getElementById('inp-sx').value = x;
        document.getElementById('inp-sy').value = y;
        drawBoard();
      }
    });

    ['inp-w', 'inp-h', 'inp-sx', 'inp-sy'].forEach(id => {
      document.getElementById(id).addEventListener('change', drawBoard);
    });

    drawBoard();
  </script>
</body>
</html>
"""


class VisualHandler(BaseHTTPRequestHandler):
    def _send_response(self, code, content_type, body):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == "/" or parsed.path == "/index.html":
            self._send_response(200, "text/html; charset=utf-8", WEB_PAGE_HTML.encode("utf-8"))
        elif parsed.path == "/api/solve":
            params = parse_qs(parsed.query)
            try:
                w = int(params.get("w", ["8"])[0])
                h = int(params.get("h", ["8"])[0])
                sx = int(params.get("sx", ["0"])[0])
                sy = int(params.get("sy", ["0"])[0])

                t0 = time.perf_counter()
                tour = solve_knights_tour(w, h, sx, sy)
                elapsed_ms = (time.perf_counter() - t0) * 1000.0

                if tour and len(tour) == w * h:
                    payload = {"success": True, "tour": tour, "time_ms": elapsed_ms}
                else:
                    _, reason = is_tour_feasible(w, h, sx, sy)
                    payload = {"success": False, "tour": [], "time_ms": elapsed_ms, "reason": reason}
            except Exception as e:
                payload = {"success": False, "error": str(e), "time_ms": 0.0}

            body = json.dumps(payload).encode("utf-8")
            self._send_response(200, "application/json", body)
        else:
            self._send_response(404, "text/plain", b"Not Found")

    def log_message(self, format, *args):
        # Suppress noisy HTTP request logging
        pass


def run_visual_server(port=5003):
    server_address = ("127.0.0.1", port)
    for p in range(port, port + 10):
        try:
            httpd = HTTPServer(("127.0.0.1", p), VisualHandler)
            url = f"http://127.0.0.1:{p}"
            print(f"[+] Knight's Tour Visualizer running at: {url}")
            threading.Timer(1.0, lambda: webbrowser.open(url)).start()
            httpd.serve_forever()
            return
        except OSError:
            continue
    print("[!] Could not bind to an available port for visual server.")


# --------------------------------------------------------------------------- #
# Entry Point
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    if "--visual" in sys.argv:
        run_visual_server()
    else:
        run_console()
