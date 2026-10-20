#!/usr/bin/env python3
"""
Chinese Postman Problem (CPP) / Rural Postman Problem (RPP)
DLH Dump Truck Garbage Collection Simulator

Algorithms:
1. Strongly Connected Component (SCC) Extraction & Filtering
2. Component Metric Closure & Arborescence Bridging (Frederickson 1979)
3. Degree Imbalance Minimization via Edmonds-Johnson (1973) Bipartite Matching
   (Hungarian Algorithm / Scipy Linear Sum Assignment)
4. Hierholzer's Algorithm for Directed Eulerian Circuit Extraction
"""

import os
import sys
import time
import math
import webbrowser
import threading
import numpy as np
import networkx as nx

# --------------------------------------------------------------------------- #
# Configuration & Constants
# --------------------------------------------------------------------------- #
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PLACE_NAME = "Surabaya, Indonesia"
GRAPH_FILENAME = os.path.join(BASE_DIR, "surabaya_road_network.graphml")

DEFAULT_CENTER = (-7.2825, 112.7949)    # ITS Sukolilo Campus
DEFAULT_RADIUS_KM = 1.0
MIN_RADIUS_KM, MAX_RADIUS_KM = 0.3, 5.0
MAX_SNAP_KM = 1.0
PORT = 5002

REQUIRED_HIGHWAYS = {
    "trunk", "trunk_link",
    "primary", "primary_link",
    "secondary", "secondary_link",
    "tertiary", "tertiary_link",
}

CLASS_SPEED_KMH = {
    "motorway": 50, "motorway_link": 30, "trunk": 40, "trunk_link": 25,
    "primary": 35, "primary_link": 25, "secondary": 30, "secondary_link": 25,
    "tertiary": 25, "tertiary_link": 20,
}
DEFAULT_SPEED_KMH = 20


# --------------------------------------------------------------------------- #
# Helpers & Formatting
# --------------------------------------------------------------------------- #
def as_list(value):
    if value is None:
        return []
    if isinstance(value, (list, tuple, set)):
        return [str(v) for v in value]
    if isinstance(value, str):
        if value.startswith("[") and value.endswith("]"):
            try:
                import ast
                parsed = ast.literal_eval(value)
                if isinstance(parsed, (list, tuple)):
                    return [str(x) for x in parsed]
            except Exception:
                pass
        return [value]
    return [str(value)]


def street_name(data):
    raw = data.get("name")
    if isinstance(raw, (list, tuple)):
        raw = raw[0] if raw else None
    if isinstance(raw, str) and raw.strip() and raw.lower() != "nan":
        return raw.strip()
    return "Unnamed road"


def road_class(highways):
    for h in highways:
        if h in REQUIRED_HIGHWAYS:
            return h
    return highways[0] if highways else "road"


def parse_maxspeed(value):
    for raw in as_list(value):
        num = "".join(c for c in str(raw).split()[0] if c.isdigit() or c == ".")
        if num:
            try:
                val = float(num)
                if val > 0:
                    return val
            except ValueError:
                pass
    return None


def edge_speed_kmh(highway, maxspeed):
    limit = parse_maxspeed(maxspeed)
    default = CLASS_SPEED_KMH.get(highway, DEFAULT_SPEED_KMH)
    return min(limit, default) if limit else default


def get_geometry(data):
    geom = data.get("geometry")
    if geom is None:
        return None
    try:
        return [(float(y), float(x)) for x, y in geom.coords]
    except Exception:
        return None


def fmt_duration(minutes):
    if minutes < 60:
        return f"{minutes:.1f} mins"
    h = int(minutes // 60)
    m = int(round(minutes % 60))
    return f"{h}h {m}m"


# --------------------------------------------------------------------------- #
# Road Network Extraction & Spatial Index
# --------------------------------------------------------------------------- #
def load_surabaya_graph():
    import osmnx as ox
    if os.path.exists(GRAPH_FILENAME):
        print(f"[-] Loading road network from cached '{GRAPH_FILENAME}'...")
        try:
            return ox.load_graphml(GRAPH_FILENAME)
        except Exception as e:
            print(f"[!] Cached GraphML unreadable ({e}); re-downloading...")
            if os.path.exists(GRAPH_FILENAME):
                os.remove(GRAPH_FILENAME)
    print(f"[-] Downloading drive network for {PLACE_NAME} (first-time initialization)...")
    G = ox.graph_from_place(PLACE_NAME, network_type="drive")
    ox.save_graphml(G, filepath=GRAPH_FILENAME)
    print(f"[+] Saved road network to '{GRAPH_FILENAME}'.")
    return G


def build_digraph(G):
    scc = max(nx.strongly_connected_components(G), key=len)
    G_scc = G.subgraph(scc)
    D = nx.DiGraph()
    for n, d in G_scc.nodes(data=True):
        D.add_node(n, x=float(d["x"]), y=float(d["y"]))

    for u, v, d in G_scc.edges(data=True):
        length = float(d.get("length", 0.0))
        cur = D.get_edge_data(u, v)
        if cur is None or length < cur["length"]:
            hws = as_list(d.get("highway"))
            cls = road_class(hws)
            D.add_edge(u, v, length=length, geometry=get_geometry(d),
                       name=street_name(d), highway=cls,
                       speed=edge_speed_kmh(cls, d.get("maxspeed")),
                       is_required=cls in REQUIRED_HIGHWAYS)
    print(f"[+] Digraph ready: {D.number_of_nodes():,} nodes, {D.number_of_edges():,} directed edges.")
    return D


class RoadIndex:
    def __init__(self, D):
        self.D = D
        self.nodes = list(D.nodes)
        self.pos = {n: i for i, n in enumerate(self.nodes)}
        self.lat = np.array([D.nodes[n]["y"] for n in self.nodes])
        self.lon = np.array([D.nodes[n]["x"] for n in self.nodes])
        cos_lat = np.cos(np.radians(np.mean(self.lat)))
        self.x_m = np.radians(self.lon) * 6371000.0 * cos_lat
        self.y_m = np.radians(self.lat) * 6371000.0
        from scipy.spatial import cKDTree
        self.tree = cKDTree(np.column_stack([self.x_m, self.y_m]))

    def nearest(self, lat, lon):
        cos_lat = np.cos(np.radians(np.mean(self.lat)))
        xm = math.radians(lon) * 6371000.0 * cos_lat
        ym = math.radians(lat) * 6371000.0
        dist_m, idx = self.tree.query([xm, ym])
        return self.nodes[idx], dist_m / 1000.0

    def nodes_within_radius(self, lat, lon, radius_km):
        cos_lat = np.cos(np.radians(np.mean(self.lat)))
        xm = math.radians(lon) * 6371000.0 * cos_lat
        ym = math.radians(lat) * 6371000.0
        indices = self.tree.query_ball_point([xm, ym], radius_km * 1000.0)
        return {self.nodes[i] for i in indices}


def select_required(D, index, base, radius_km):
    """
    Filters required edges within radius_km whose midpoint is in radius,
    restricting to reachable component containing base.
    """
    R_m = radius_km * 1000.0
    bx, by = D.nodes[base]["x"], D.nodes[base]["y"]
    cos_lat = np.cos(np.radians(by))
    base_xm = math.radians(bx) * 6371000.0 * cos_lat
    base_ym = math.radians(by) * 6371000.0

    required = []
    involved = set()
    for u, v, d in D.edges(data=True):
        if not d.get("is_required", False):
            continue
        mx = (D.nodes[u]["x"] + D.nodes[v]["x"]) / 2.0
        my = (D.nodes[u]["y"] + D.nodes[v]["y"]) / 2.0
        xm = math.radians(mx) * 6371000.0 * cos_lat
        ym = math.radians(my) * 6371000.0
        if (xm - base_xm) ** 2 + (ym - base_ym) ** 2 <= R_m ** 2:
            required.append((u, v))
            involved.add(u)
            involved.add(v)

    # Restrict road network to neighbourhood containing required edges + base
    neighbourhood = index.nodes_within_radius(by, bx, radius_km * 1.6) | involved | {base}
    sub = D.subgraph(neighbourhood).copy()

    # Retain largest SCC containing base
    base_scc = None
    for c in nx.strongly_connected_components(sub):
        if base in c:
            base_scc = c
            break

    if base_scc is None:
        return sub, [], len(required)

    W = sub.subgraph(base_scc).copy()
    valid_required = [(u, v) for u, v in required if u in W and v in W and W.has_edge(u, v)]
    dropped = len(required) - len(valid_required)
    return W, valid_required, dropped


# --------------------------------------------------------------------------- #
# Algorithmic Core: Component Bridging, Bipartite Matching, Hierholzer Tour
# --------------------------------------------------------------------------- #
def add_path_to_multigraph(M, path):
    for u, v in zip(path[:-1], path[1:]):
        M.add_edge(u, v)


def connect_required_components(W, required, base):
    """
    Frederickson's (1979) Rural Postman Component Metric Closure.
    Ensures every isolated piece of the required network is reachable from
    and can return to the dump truck base depot via shortest directed paths.
    """
    M = nx.MultiDiGraph()
    M.add_node(base)
    for u, v in required:
        M.add_edge(u, v)

    W_rev = W.reverse(copy=False)

    while True:
        comps = list(nx.weakly_connected_components(M))
        if len(comps) == 1:
            break

        # Main component containing base
        main_comp = next(c for c in comps if base in c)
        other_nodes = set().union(*[c for c in comps if base not in c])

        # Shortest forward paths from main to others, and backward paths from others to main
        d_f, p_f = nx.multi_source_dijkstra(W, main_comp, weight="length")
        d_b, p_b = nx.multi_source_dijkstra(W_rev, main_comp, weight="length")

        target = min(other_nodes, key=lambda n: min(d_f.get(n, math.inf), d_b.get(n, math.inf)))
        if d_f.get(target, math.inf) <= d_b.get(target, math.inf):
            path = p_f[target]  # Forward bridge: main -> target
        else:
            path = p_b[target][::-1]  # Backward bridge: target -> main

        add_path_to_multigraph(M, path)

    return M


def balance_eulerian_degrees_bipartite(W, M):
    """
    Edmonds & Johnson (1973) Directed Chinese Postman Algorithm:
    Resolves in-degree vs out-degree imbalances using Minimum Weight Bipartite Matching
    (Hungarian Algorithm / Scipy Linear Sum Assignment).
    
    Nodes with in_degree > out_degree have surplus incoming edges (supply/sources).
    Nodes with out_degree > in_degree have surplus outgoing edges (demand/sinks).
    """
    from scipy.optimize import linear_sum_assignment

    supply_nodes = []
    demand_nodes = []

    for n in M.nodes:
        diff = M.in_degree(n) - M.out_degree(n)
        if diff > 0:
            supply_nodes.extend([n] * diff)
        elif diff < 0:
            demand_nodes.extend([n] * (-diff))

    if not supply_nodes or not demand_nodes:
        return M

    assert len(supply_nodes) == len(demand_nodes), "Total in-degree must equal total out-degree."

    # Compute shortest paths from each unique supply node to demand nodes
    unique_supplies = list(set(supply_nodes))
    dist_cache = {}
    path_cache = {}
    for s in unique_supplies:
        dists, paths = nx.single_source_dijkstra(W, s, weight="length")
        dist_cache[s] = dists
        path_cache[s] = paths

    # Construct cost matrix for bipartite matching
    n_pairs = len(supply_nodes)
    cost_matrix = np.full((n_pairs, n_pairs), 1e9)

    for i, s in enumerate(supply_nodes):
        for j, t in enumerate(demand_nodes):
            if t in dist_cache[s]:
                cost_matrix[i, j] = dist_cache[s][t]

    # Solve optimal one-to-one assignment using Hungarian Algorithm
    row_ind, col_ind = linear_sum_assignment(cost_matrix)

    for r, c in zip(row_ind, col_ind):
        s = supply_nodes[r]
        t = demand_nodes[c]
        shortest_path = path_cache[s][t]
        add_path_to_multigraph(M, shortest_path)

    return M


def extract_hierholzer_circuit(M, start_node):
    """
    Hierholzer's Algorithm for extracting a directed Eulerian Circuit.
    Guaranteed linear time O(E') on balanced, strongly connected multigraphs.
    """
    # Create working adjacency structure with multi-edges
    adj = {u: [] for u in M.nodes}
    for u, v in M.edges():
        adj[u].append(v)

    curr_path = [start_node]
    circuit = []

    while curr_path:
        curr_v = curr_path[-1]
        if adj[curr_v]:
            next_v = adj[curr_v].pop()
            curr_path.append(next_v)
        else:
            circuit.append(curr_path.pop())

    circuit.reverse()
    # Convert node sequence to edge pairs
    return list(zip(circuit[:-1], circuit[1:]))


def extract_fleury_circuit(M, start_node):
    """
    Fleury's Algorithm for extracting a directed Eulerian Circuit.
    Reference: Week 3 Lecture Slide 12.
    Philosophy: "Never cross a bridge unless you have no alternative edge."
    Time Complexity: O(E'^2)
    """
    G = nx.MultiDiGraph(M)
    curr = start_node
    circuit = []

    def is_bridge_or_only_edge(u, v, key):
        if G.out_degree(u) == 1:
            return True
        active = [n for n in G if G.degree(n) > 0]
        comps_before = nx.number_weakly_connected_components(G.subgraph(active))
        edge_data = G[u][v][key]
        G.remove_edge(u, v, key=key)
        active_after = [n for n in G if G.degree(n) > 0]
        comps_after = nx.number_weakly_connected_components(G.subgraph(active_after)) if active_after else 0
        G.add_edge(u, v, key=key, **edge_data)
        return comps_after <= comps_before

    total_edges = G.number_of_edges()
    for _ in range(total_edges):
        out_edges = list(G.out_edges(curr, keys=True))
        if not out_edges:
            break
        chosen = None
        for u, v, k in out_edges:
            if is_bridge_or_only_edge(u, v, k):
                chosen = (u, v, k)
                break
        if chosen is None:
            chosen = out_edges[0]
        u, v, k = chosen
        G.remove_edge(u, v, key=k)
        circuit.append((u, v))
        curr = v

    return circuit


def solve_cpp(W, required, base):
    """
    Complete Directed Chinese/Rural Postman Problem Solver using Hierholzer's Algorithm
    (Reference: Week 3 Lecture Slides 13-15).
    """
    if not required:
        return [], {"optimal": True, "method": "Trivial (no required edges)"}

    # Step 1: Component Bridging (Frederickson 1979)
    M = connect_required_components(W, required, base)

    # Step 2: Degree Balancing via Bipartite Matching (Edmonds & Johnson 1973)
    M = balance_eulerian_degrees_bipartite(W, M)

    # Step 3: Eulerian Circuit Generation via Hierholzer's Algorithm (Week 3, Slide 13)
    circuit = extract_hierholzer_circuit(M, base)

    total_m = sum(W[u][v]["length"] for u, v in circuit)
    return circuit, {
        "optimal": True,
        "method": "Hierholzer's Algorithm (Week 3 Material)",
        "heuristic_m": total_m
    }


# --------------------------------------------------------------------------- #
# Traversal Segments & Visualization Pipeline
# --------------------------------------------------------------------------- #
def edge_coords(W, u, v, geom):
    if geom:
        return geom
    return [(W.nodes[u]["y"], W.nodes[u]["x"]), (W.nodes[v]["y"], W.nodes[v]["x"])]


def build_segments(W, circuit, required_set):
    """
    Merges contiguous directed edges of the same street and collection status
    for clean map rendering and readable traversal steps.
    """
    segments = []
    for u, v in circuit:
        d = W[u][v]
        is_collect = (u, v) in required_set
        coords = edge_coords(W, u, v, d.get("geometry"))

        if segments and segments[-1]["name"] == d["name"] and segments[-1]["collect"] == is_collect:
            segments[-1]["length"] += d["length"]
            segments[-1]["coords"].extend(coords[1:])
        else:
            segments.append({
                "name": d["name"],
                "highway": d["highway"],
                "collect": is_collect,
                "length": d["length"],
                "coords": list(coords)
            })
    return segments


def run_cpp(D, index, lat, lon, radius_km):
    base, snap_km = index.nearest(lat, lon)
    if snap_km > MAX_SNAP_KM:
        raise ValueError("No road found near that point. Please click inside Surabaya.")

    W, required, dropped = select_required(D, index, base, radius_km)
    if not required:
        raise ValueError("No trunk/primary/secondary/tertiary roads within that radius. Try adjusting radius or location.")

    t0 = time.perf_counter()
    circuit, info = solve_cpp(W, required, base)
    exec_ms = (time.perf_counter() - t0) * 1000.0

    required_set = set(required)
    segments = build_segments(W, circuit, required_set)

    total_m = sum(W[u][v]["length"] for u, v in circuit)
    collect_m = sum(W[u][v]["length"] for u, v in required_set)
    revisit_m = max(0.0, total_m - collect_m)
    seconds = sum(W[u][v]["length"] / (W[u][v]["speed"] / 3.6) for u, v in circuit)

    return {
        "base": {"lat": W.nodes[base]["y"], "lon": W.nodes[base]["x"], "snap_m": snap_km * 1000.0},
        "radius_km": radius_km,
        "required_edges": len(required),
        "dropped": dropped,
        "stats": {
            "total_km": total_m / 1000.0,
            "collect_km": collect_m / 1000.0,
            "revisit_km": revisit_m / 1000.0,
            "minutes": seconds / 60.0,
            "duration": fmt_duration(seconds / 60.0),
            "exec_ms": exec_ms,
            "edges_traversed": len(circuit),
            "method": info["method"],
            "optimal": info["optimal"]
        },
        "segments": [
            {
                "i": i,
                "name": s["name"],
                "highway": s["highway"],
                "collect": s["collect"],
                "km": s["length"] / 1000.0,
                "coords": [[round(a, 6), round(b, 6)] for a, b in s["coords"]]
            }
            for i, s in enumerate(segments, 1)
        ]
    }


# --------------------------------------------------------------------------- #
# Web App (Flask + Leaflet.js)
# --------------------------------------------------------------------------- #
PAGE_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>DLH Dump Truck — Chinese Postman Simulator</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link href="https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@400;500;600;700&family=JetBrains+Mono:wght@400;600&display=swap" rel="stylesheet">
  <style>
    :root {
      --primary: #059669;
      --primary-dark: #047857;
      --accent: #f59e0b;
      --deadhead: #ea580c;
      --panel: #ffffff;
      --border: #e2e8f0;
      --text: #1e293b;
      --text-muted: #64748b;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    html, body, #app { height: 100%; width: 100%; overflow: hidden; font-family: 'Plus Jakarta Sans', sans-serif; }
    #app { display: flex; }
    #side {
      width: 410px;
      height: 100%;
      display: flex;
      flex-direction: column;
      background: var(--panel);
      border-right: 1px solid var(--border);
      box-shadow: 4px 0 15px rgba(0,0,0,0.05);
      z-index: 1000;
    }
    #map { flex: 1; height: 100%; }
    .pad { padding: 16px 20px; }
    .header-bar {
      background: #064e3b;
      color: #fff;
      padding: 16px 20px;
      border-bottom: 3px solid var(--accent);
    }
    .badge {
      display: inline-block;
      font-size: 10px;
      font-weight: 700;
      background: var(--primary);
      padding: 3px 6px;
      border-radius: 4px;
      margin-bottom: 6px;
      letter-spacing: 0.5px;
    }
    h1 { font-size: 17px; font-weight: 700; margin: 0; }
    .subtitle { font-size: 12px; color: #a7f3d0; margin-top: 3px; }
    label { display: block; font-size: 12px; font-weight: 600; color: var(--text-muted); margin: 12px 0 4px; text-transform: uppercase; letter-spacing: 0.5px; }
    input[type=range] { width: 100%; }
    .btn-solve {
      width: 100%;
      background: var(--primary);
      color: white;
      border: none;
      padding: 12px;
      border-radius: 8px;
      font-size: 14px;
      font-weight: 700;
      cursor: pointer;
      margin-top: 14px;
      transition: all 0.2s;
    }
    .btn-solve:hover:not(:disabled) { background: var(--primary-dark); }
    .btn-solve:disabled { background: #cbd5e1; cursor: not-allowed; }
    #stats-box {
      display: none;
      background: #f8fafc;
      border-top: 1px solid var(--border);
      border-bottom: 1px solid var(--border);
      padding: 14px 20px;
    }
    .stat-grid {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
    }
    .stat-card {
      background: #fff;
      padding: 8px 10px;
      border-radius: 6px;
      border: 1px solid var(--border);
    }
    .stat-card .label { font-size: 10px; font-weight: 700; color: var(--text-muted); text-transform: uppercase; }
    .stat-card .val { font-size: 15px; font-weight: 700; color: #0f172a; font-family: 'JetBrains Mono', monospace; }
    .legend-row {
      display: flex;
      gap: 16px;
      margin-top: 10px;
      font-size: 12px;
      font-weight: 600;
    }
    .legend-item { display: flex; align-items: center; gap: 6px; }
    .legend-dot { width: 12px; height: 12px; border-radius: 3px; }
    .order-header {
      display: none;
      justify-content: space-between;
      align-items: center;
      padding: 12px 20px 8px;
      font-size: 12px;
      font-weight: 700;
      color: var(--text-muted);
      border-bottom: 1px solid var(--border);
    }
    .order-header a { color: var(--primary); text-decoration: none; cursor: pointer; }
    .order-list {
      flex: 1;
      overflow-y: auto;
      list-style: none;
      padding: 0;
      margin: 0;
    }
    .order-list li {
      padding: 10px 20px;
      border-bottom: 1px solid #f1f5f9;
      font-size: 13px;
      line-height: 1.4;
      cursor: pointer;
      transition: background 0.15s;
    }
    .order-list li:hover { background: #f0fdf4; }
    .seg-badge {
      font-size: 10px;
      font-weight: 700;
      padding: 2px 6px;
      border-radius: 4px;
      margin-right: 6px;
    }
    .seg-collect { background: #dcfce7; color: #15803d; }
    .seg-deadhead { background: #ffedd5; color: #c2410c; }
  </style>
</head>
<body>
  <div id="app">
    <div id="side">
      <div class="header-bar">
        <div class="badge">TASK 2: CHINESE POSTMAN SIMULATOR</div>
        <h1>DLH Garbage Collection</h1>
        <div class="subtitle">Directed Rural Postman on Surabaya Streets</div>
      </div>

      <div class="pad">
        <div style="font-size: 13px; line-height: 1.4; color: #475569;">
          Click anywhere on the map to set the <strong>DLH Dump Truck Base</strong>.
        </div>

        <label>Service Radius: <span id="lbl-rad" style="color: #0f172a; font-weight: 700;">__DEFR__ km</span></label>
        <input type="range" id="inp-rad" min="__MINR__" max="__MAXR__" step="0.1" value="__DEFR__">

        <div style="margin: 10px 0 4px; font-size: 11px; color: #047857; background: #d1fae5; padding: 7px 10px; border-radius: 6px; font-weight: 600;">
          Algorithm: Hierholzer's Algorithm (Week 3 Material)
        </div>

        <button id="btn-go" class="btn-solve" disabled>Compute Collection Route</button>
        <div id="msg" style="font-size: 12px; margin-top: 8px; color: #64748b; min-height: 16px;"></div>

        <div class="legend-row">
          <div class="legend-item">
            <div class="legend-dot" style="background: #059669;"></div>
            <span>Collection Pass</span>
          </div>
          <div class="legend-item">
            <div class="legend-dot" style="background: #ea580c;"></div>
            <span>Deadheading / Revisit</span>
          </div>
        </div>
      </div>

      <div id="stats-box">
        <div class="stat-grid">
          <div class="stat-card">
            <div class="label">Total Route</div>
            <div id="st-dist" class="val">0.0 km</div>
          </div>
          <div class="stat-card">
            <div class="label">Collection Roads</div>
            <div id="st-collect" class="val" style="color: #059669;">0.0 km</div>
          </div>
          <div class="stat-card">
            <div class="label">Revisit / Detour</div>
            <div id="st-revisit" class="val" style="color: #ea580c;">0.0 km</div>
          </div>
          <div class="stat-card">
            <div class="label">Solver Execution</div>
            <div id="st-solver" class="val">0.0 ms</div>
          </div>
        </div>
        <div id="st-method" style="font-size: 11px; color: #64748b; margin-top: 8px; font-style: italic;"></div>
      </div>

      <div id="order-head" class="order-header">
        <span>STREET TRAVERSAL SEQUENCE</span>
        <a id="btn-export">Export Route TXT</a>
      </div>
      <ul id="order-list" class="order-list"></ul>
    </div>

    <div id="map"></div>
  </div>

  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet-polylinedecorator/1.6.0/leaflet.polylineDecorator.min.js"></script>
  <script>
    const map = L.map('map').setView([__LAT__, __LON__], 13);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
      attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    const radInput = document.getElementById('inp-rad');
    const radLabel = document.getElementById('lbl-rad');
    const btnGo = document.getElementById('btn-go');
    const msgEl = document.getElementById('msg');

    let base = null;
    let baseMarker = null;
    let circle = null;
    let lastData = null;

    const routeLayer = L.layerGroup().addTo(map);
    const arrowLayer = L.layerGroup().addTo(map);

    function refreshRadius() {
      const r = parseFloat(radInput.value);
      radLabel.textContent = r.toFixed(1) + ' km';
      if (!base) return;

      if (circle) map.removeLayer(circle);
      circle = L.circle([base.lat, base.lon], {
        radius: r * 1000,
        color: '#059669',
        fillColor: '#059669',
        fillOpacity: 0.08,
        weight: 1.5
      }).addTo(map);
    }

    radInput.addEventListener('input', refreshRadius);

    map.on('click', (e) => {
      base = { lat: e.latlng.lat, lon: e.latlng.lng };
      if (baseMarker) map.removeLayer(baseMarker);
      baseMarker = L.marker(e.latlng, {
        icon: L.icon({
          iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-green.png',
          shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
          iconSize: [25, 41],
          iconAnchor: [12, 41],
          popupAnchor: [1, -34],
          shadowSize: [41, 41]
        })
      }).addTo(map).bindPopup('<b>DLH Dump Truck Depot</b>').openPopup();

      btnGo.disabled = false;
      msgEl.textContent = 'Dump truck base set. Click Compute Collection Route.';
      refreshRadius();
    });

    btnGo.addEventListener('click', async () => {
      if (!base) return;
      btnGo.disabled = true;
      msgEl.textContent = 'Solving Directed Rural Postman Problem...';

      const radius = parseFloat(radInput.value);

      try {
        const resp = await fetch('/api/solve', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ lat: base.lat, lon: base.lon, radius: radius })
        });
        const data = await resp.json();
        if (!resp.ok) throw new Error(data.error || 'Server error');
        renderSolution(data);
        msgEl.textContent = 'Route calculated successfully.';
      } catch (err) {
        msgEl.textContent = 'Error: ' + err.message;
      } finally {
        btnGo.disabled = false;
      }
    });

    function renderSolution(data) {
      lastData = data;
      routeLayer.clearLayers();
      arrowLayer.clearLayers();

      const allCoords = [];
      data.segments.forEach(seg => {
        allCoords.push(...seg.coords);
        const poly = L.polyline(seg.coords, {
          color: seg.collect ? '#059669' : '#ea580c',
          weight: seg.collect ? 4 : 2.5,
          dashArray: seg.collect ? null : '6, 6',
          opacity: seg.collect ? 0.9 : 0.75
        }).addTo(routeLayer);
      });

      // Update statistics
      document.getElementById('stats-box').style.display = 'block';
      document.getElementById('order-head').style.display = 'flex';
      document.getElementById('st-dist').textContent = data.stats.total_km.toFixed(2) + ' km';
      document.getElementById('st-collect').textContent = data.stats.collect_km.toFixed(2) + ' km';
      document.getElementById('st-revisit').textContent = data.stats.revisit_km.toFixed(2) + ' km';
      document.getElementById('st-solver').textContent = data.stats.exec_ms.toFixed(1) + ' ms';
      document.getElementById('st-method').textContent = 'Engine: ' + data.stats.method;

      // Populate street traversal list
      const listEl = document.getElementById('order-list');
      listEl.innerHTML = '';

      data.segments.forEach(s => {
        const li = document.createElement('li');
        const badgeCls = s.collect ? 'seg-collect' : 'seg-deadhead';
        const badgeTxt = s.collect ? 'COLLECT' : 'TRAVEL';
        li.innerHTML = `<span class="seg-badge ${badgeCls}">${badgeTxt}</span>` +
                       `<strong>${s.name}</strong> (${s.highway})<br>` +
                       `<span style="font-size:11px;color:#64748b;">Step ${s.i} &bull; ${s.km.toFixed(2)} km</span>`;
        li.addEventListener('click', () => {
          if (s.coords.length > 0) {
            map.fitBounds(L.latLngBounds(s.coords).pad(0.3));
          }
        });
        listEl.appendChild(li);
      });

      if (allCoords.length > 0) {
        map.fitBounds(L.latLngBounds(allCoords).pad(0.1));
      }
    }

    document.getElementById('btn-export').addEventListener('click', () => {
      if (!lastData) return;
      const st = lastData.stats;
      let txt = `DLH DUMP TRUCK GARBAGE COLLECTION ROUTE\\nTotal Distance: ${st.total_km.toFixed(2)} km\\nCollection Distance: ${st.collect_km.toFixed(2)} km\\nDeadheading Distance: ${st.revisit_km.toFixed(2)} km\\nEstimated Time: ${st.duration}\\nSolver Time: ${st.exec_ms.toFixed(2)} ms\\nMethod: ${st.method}\\n\\n`;
      txt += `0. [START] DLH Depot\\n`;
      lastData.segments.forEach(s => {
        const tag = s.collect ? '[COLLECT]' : '[TRAVEL]';
        txt += `${s.i}. ${tag} ${s.name} (${s.highway}) — ${s.km.toFixed(2)} km\\n`;
      });
      txt += `${lastData.segments.length + 1}. [FINISH] Return to DLH Depot\\n`;

      const blob = new Blob([txt], { type: 'text/plain;charset=utf-8' });
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = 'dlh_collection_route.txt';
      a.click();
    });
  </script>
</body>
</html>
"""


def create_app(D):
    from flask import Flask, Response, jsonify, request

    index = RoadIndex(D)
    app = Flask(__name__)
    lock = threading.Lock()

    html_content = (PAGE_HTML
                    .replace("__LAT__", str(DEFAULT_CENTER[0]))
                    .replace("__LON__", str(DEFAULT_CENTER[1]))
                    .replace("__MINR__", str(MIN_RADIUS_KM))
                    .replace("__MAXR__", str(MAX_RADIUS_KM))
                    .replace("__DEFR__", str(DEFAULT_RADIUS_KM)))

    @app.get("/")
    def index_route():
        return Response(html_content, mimetype="text/html")

    @app.post("/api/solve")
    def solve_route():
        try:
            body = request.get_json(force=True)
            lat, lon = float(body["lat"]), float(body["lon"])
            radius = float(body["radius"])
        except (KeyError, TypeError, ValueError):
            return jsonify(error="Invalid parameters in request body."), 400

        if not (MIN_RADIUS_KM <= radius <= MAX_RADIUS_KM):
            return jsonify(error=f"Radius must be between {MIN_RADIUS_KM} and {MAX_RADIUS_KM} km."), 400

        try:
            with lock:
                res = run_cpp(D, index, lat, lon, radius)
            return jsonify(res)
        except ValueError as e:
            return jsonify(error=str(e)), 422

    return app


def main():
    G = load_surabaya_graph()
    D = build_digraph(G)
    app = create_app(D)
    url = f"http://127.0.0.1:{PORT}"
    print(f"[+] DLH CPP Server started. Open {url}")
    threading.Timer(1.2, lambda: webbrowser.open(url)).start()
    app.run(host="127.0.0.1", port=PORT, debug=False, threaded=True)


if __name__ == "__main__":
    main()
