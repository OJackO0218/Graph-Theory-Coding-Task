#!/usr/bin/env python3
"""
Travelling Salesman Problem (TSP) — SPPG Mealbox Delivery Simulator
Algorithms:
1. Farthest Insertion Heuristic (Global hull construction)
2. Directed 2-Opt Local Search (Asymmetric edge reversal)
3. Simulated Annealing Metaheuristic (Stochastic local search)
4. Held-Karp Dynamic Programming (Exact bitmask DP, O(n^2 2^n))
"""

import os
import sys
import time
import json
import math
import webbrowser
import threading
import itertools
import numpy as np
import networkx as nx

# --------------------------------------------------------------------------- #
# Configuration & Constants
# --------------------------------------------------------------------------- #
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PLACE_NAME = "Surabaya, Indonesia"
GRAPH_FILENAME = os.path.join(BASE_DIR, "surabaya_road_network.graphml")
SCHOOLS_FILENAME = os.path.join(BASE_DIR, "surabaya_schools.json")

DEFAULT_CENTER = (-7.2825, 112.7949)    # ITS Sukolilo Campus
DEFAULT_RADIUS_KM = 3.0
MIN_RADIUS_KM, MAX_RADIUS_KM = 0.5, 10.0
MAX_SNAP_KM = 1.0                       # Maximum allowable distance to snap to a road node
PORT = 5001

CLASS_SPEED_KMH = {
    "motorway": 50, "motorway_link": 30, "trunk": 40, "trunk_link": 25,
    "primary": 35, "primary_link": 25, "secondary": 30, "secondary_link": 25,
    "tertiary": 25, "tertiary_link": 20,
}
DEFAULT_SPEED_KMH = 20


# --------------------------------------------------------------------------- #
# Helper Utilities
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


def haversine_km(lat1, lon1, lat2, lon2):
    R = 6371.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2.0) ** 2 +
         math.cos(phi1) * math.cos(phi2) * (math.sin(dlam / 2.0) ** 2))
    return 2.0 * R * math.asin(math.sqrt(max(0.0, min(1.0, a))))


# --------------------------------------------------------------------------- #
# Data Loading & Road Network
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
            hw = as_list(d.get("highway"))
            cls = hw[0] if hw else "road"
            D.add_edge(u, v, length=length, geometry=get_geometry(d),
                       speed=edge_speed_kmh(cls, d.get("maxspeed")), highway=cls)
    print(f"[+] Digraph ready: {D.number_of_nodes():,} nodes, {D.number_of_edges():,} directed edges.")
    return D


def load_schools():
    raw = None
    if os.path.exists(SCHOOLS_FILENAME):
        print(f"[-] Loading schools from '{SCHOOLS_FILENAME}'...")
        with open(SCHOOLS_FILENAME, "r", encoding="utf-8") as f:
            raw = json.load(f)
    else:
        import osmnx as ox
        print("[-] Fetching school locations from OpenStreetMap (one-time)...")
        gdf = ox.features_from_place(PLACE_NAME, tags={"amenity": "school"})
        gdf = gdf[~gdf.geometry.is_empty]
        cent = gdf.geometry.to_crs(epsg=32749).centroid.to_crs(epsg=4326)
        raw = [{"name": gdf.iloc[i].get("name"), "lat": float(c.y), "lon": float(c.x)}
               for i, c in enumerate(cent)]
        with open(SCHOOLS_FILENAME, "w", encoding="utf-8") as f:
            json.dump(raw, f, ensure_ascii=False, indent=2)
        print(f"[+] Saved to '{SCHOOLS_FILENAME}'.")

    schools, seen = [], set()
    for s in raw:
        name = s.get("name")
        if not isinstance(name, str) or not name.strip() or name.lower() == "nan":
            name = "Unnamed school"
        key = (name, round(float(s["lat"]), 5), round(float(s["lon"]), 5))
        if key in seen:
            continue
        seen.add(key)
        schools.append({"name": name, "lat": float(s["lat"]), "lon": float(s["lon"])})
    print(f"[+] {len(schools):,} schools loaded.")
    return schools


class RoadNet:
    def __init__(self, D):
        import scipy.sparse as sp
        self.D = D
        self.ids = list(D.nodes)
        self.pos = {n: i for i, n in enumerate(self.ids)}
        self.lat = np.array([D.nodes[n]["y"] for n in self.ids])
        self.lon = np.array([D.nodes[n]["x"] for n in self.ids])
        cos_lat = np.cos(np.radians(np.mean(self.lat)))
        self.x_m = np.radians(self.lon) * 6371000.0 * cos_lat
        self.y_m = np.radians(self.lat) * 6371000.0
        from scipy.spatial import cKDTree
        self.tree = cKDTree(np.column_stack([self.x_m, self.y_m]))

        rows, cols, data = [], [], []
        for u, v, d in D.edges(data=True):
            rows.append(self.pos[u])
            cols.append(self.pos[v])
            data.append(d["length"])
        n = len(self.ids)
        self.csr = sp.csr_matrix((data, (rows, cols)), shape=(n, n))

    def nearest(self, lat, lon):
        cos_lat = np.cos(np.radians(np.mean(self.lat)))
        xm = math.radians(lon) * 6371000.0 * cos_lat
        ym = math.radians(lat) * 6371000.0
        dist_m, idx = self.tree.query([xm, ym])
        return idx, dist_m / 1000.0


# --------------------------------------------------------------------------- #
# TSP Solvers: Farthest Insertion, Directed 2-Opt, Simulated Annealing, Held-Karp
# --------------------------------------------------------------------------- #
def tour_cost(d, tour):
    n = len(tour)
    return sum(d[tour[i]][tour[(i + 1) % n]] for i in range(n))


# --------------------------------------------------------------------------- #
# TSP Solvers: Minimum Spanning Tree (Prim / Kruskal) + DFS Shortcut (Week 2)
# and Bellman-Held-Karp Exact DP (Week 4 Hamiltonian Tour)
# --------------------------------------------------------------------------- #
def tour_cost(d, tour):
    n = len(tour)
    return sum(d[tour[i]][tour[(i + 1) % n]] for i in range(n))


def prim_mst(d):
    """
    Prim's Algorithm for Minimum Spanning Tree (MST)
    Reference: Week 2 Lecture Slide 18-19.
    Starts at vertex 0 and iteratively grows the tree by adding the minimum weight
    edge connecting a vertex in the tree to a vertex outside the tree.
    """
    n = len(d)
    in_tree = [False] * n
    in_tree[0] = True
    adj = {i: [] for i in range(n)}

    for _ in range(n - 1):
        min_w = float("inf")
        best_u, best_v = -1, -1
        for u in range(n):
            if in_tree[u]:
                for v in range(n):
                    if not in_tree[v]:
                        w = min(d[u][v], d[v][u])
                        if w < min_w:
                            min_w = w
                            best_u, best_v = u, v
        if best_v != -1:
            in_tree[best_v] = True
            adj[best_u].append(best_v)
            adj[best_v].append(best_u)
    return adj


def kruskal_mst(d):
    """
    Kruskal's Algorithm for Minimum Spanning Tree (MST)
    Reference: Week 2 Lecture Slide 20-21.
    Sorts all candidate edges in ascending weight order and adds edges that do
    not create cycles using Disjoint Set (Union-Find).
    """
    n = len(d)
    parent = list(range(n))

    def find(i):
        if parent[i] == i:
            return i
        parent[i] = find(parent[i])
        return parent[i]

    def union(i, j):
        root_i = find(i)
        root_j = find(j)
        if root_i != root_j:
            parent[root_i] = root_j
            return True
        return False

    edges = []
    for i in range(n):
        for j in range(i + 1, n):
            w = min(d[i][j], d[j][i])
            edges.append((w, i, j))
    edges.sort()

    adj = {i: [] for i in range(n)}
    edges_count = 0
    for w, u, v in edges:
        if union(u, v):
            adj[u].append(v)
            adj[v].append(u)
            edges_count += 1
            if edges_count == n - 1:
                break
    return adj


def dfs_spanning_tree_shortcut(adj, start=0):
    """
    Depth-First Search (DFS) Tree Traversal for Hamiltonian Path Shortcut
    Reference: Week 2 Lecture Slide 13-14 and Week 4 Slide 5.
    Traverses the Minimum Spanning Tree in depth-first preorder, skipping
    previously visited vertices to produce a valid Hamiltonian tour.
    """
    visited = set()
    tour = []

    def dfs(u):
        visited.add(u)
        tour.append(u)
        for v in sorted(adj[u]):
            if v not in visited:
                dfs(v)

    dfs(start)
    return tour


def tsp_mst_approximation(d, method="prim"):
    """
    Classic Metric TSP 2-Approximation using MST + DFS Shortcut.
    Combines Prim's / Kruskal's MST (Week 2) with DFS Traversal (Week 2).
    """
    n = len(d)
    if n <= 2:
        return list(range(n))

    if method == "kruskal":
        mst = kruskal_mst(d)
    else:
        mst = prim_mst(d)

    tour = dfs_spanning_tree_shortcut(mst, start=0)
    return tour


def tsp_directed_2opt(d, tour):
    """
    Directed 2-Opt Local Search.
    Evaluates segment reversal accounting for asymmetric directed costs:
    Reversing tour[i:j+1] reverses intermediate edges tour[k] -> tour[k-1].
    Iterates until no improving 2-opt move is found.
    """
    tour = list(tour)
    n = len(tour)
    if n <= 3:
        return tour

    improved = True
    while improved:
        improved = False
        for i in range(1, n - 1):
            for j in range(i + 1, n):
                prev_i = tour[i - 1]
                next_j = tour[(j + 1) % n]
                node_i = tour[i]
                node_j = tour[j]

                # Old edges cost
                old_cost = d[prev_i][node_i] + d[node_j][next_j]
                for k in range(i, j):
                    old_cost += d[tour[k]][tour[k + 1]]

                # New edges cost with segment reversed
                new_cost = d[prev_i][node_j] + d[node_i][next_j]
                for k in range(j, i, -1):
                    new_cost += d[tour[k]][tour[k - 1]]

                if new_cost < old_cost - 1e-9:
                    tour[i:j + 1] = reversed(tour[i:j + 1])
                    improved = True
                    break
            if improved:
                break
    return tour


def tsp_held_karp_exact(d, time_limit_s=25.0):
    """
    Bellman-Held-Karp Dynamic Programming algorithm for exact Asymmetric TSP.
    State: DP(mask, u) = minimum cost to visit vertices in 'mask' ending at u.
    Complexity: O(n^2 * 2^n). Exact and mathematically optimal for n <= 20.
    """
    n = len(d)
    if n <= 2:
        return list(range(n)), True

    # Check timeout/size guard
    if n > 20:
        return None  # Exceeds realistic exact DP table memory/time

    start_time = time.perf_counter()
    memo = {}

    # Base cases: from start 0 to every node i
    for i in range(1, n):
        memo[(1 << i, i)] = (d[0][i], 0)

    for mask_size in range(2, n):
        if time.perf_counter() - start_time > time_limit_s:
            return None  # Timeout fallback

        for comb in itertools.combinations(range(1, n), mask_size):
            mask = sum(1 << i for i in comb)
            for u in comb:
                prev_mask = mask ^ (1 << u)
                best_c = float("inf")
                best_p = None
                for v in comb:
                    if v == u:
                        continue
                    prev_entry = memo.get((prev_mask, v))
                    if prev_entry is not None:
                        c = prev_entry[0] + d[v][u]
                        if c < best_c:
                            best_c = c
                            best_p = v
                if best_p is not None:
                    memo[(mask, u)] = (best_c, best_p)

    full_mask = (1 << n) - 2  # Bits 1 through n-1
    best_total = float("inf")
    last_node = None

    for u in range(1, n):
        entry = memo.get((full_mask, u))
        if entry is not None:
            c = entry[0] + d[u][0]
            if c < best_total:
                best_total = c
                last_node = u

    if last_node is None:
        return None

    # Reconstruct tour backwards
    curr = last_node
    curr_mask = full_mask
    tour = []
    while curr != 0:
        tour.append(curr)
        next_curr = memo[(curr_mask, curr)][1]
        curr_mask = curr_mask ^ (1 << curr)
        curr = next_curr

    tour.append(0)
    tour.reverse()
    return tour, True


def solve_tsp(d):
    """
    Solves TSP using Prim's Algorithm MST + DFS Tree Shortcut (Week 2 Material):
    1. Computes Minimum Spanning Tree using Prim's Algorithm (Week 2, Slide 18-19).
    2. Traverses the MST in preorder using Depth-First Search (DFS, Week 2, Slide 13-14).
    3. Shortcuts visited vertices to form a closed Hamiltonian cycle (Week 4, Slide 5).
    4. Polishes the directed tour with 2-Opt local search.
    """
    n = len(d)
    if n == 1:
        return [0], {"optimal": True, "method": "Trivial single stop", "cost_m": 0.0}

    mst_tour = tsp_mst_approximation(d, method="prim")
    opt_tour = tsp_directed_2opt(d, mst_tour)
    cost = tour_cost(d, opt_tour)

    return opt_tour, {
        "optimal": False,
        "method": "Prim's Algorithm MST + DFS Tree Traversal (Week 2)",
        "cost_m": cost
    }


# --------------------------------------------------------------------------- #
# Pipeline: Road Graph Snapping, Dijkstra Matrix & Route Expansion
# --------------------------------------------------------------------------- #
def edge_coords(D, u, v, geom):
    if geom:
        return geom
    return [(D.nodes[u]["y"], D.nodes[u]["x"]), (D.nodes[v]["y"], D.nodes[v]["x"])]


def run_tsp(net, schools, lat, lon, radius_km):
    from scipy.sparse.csgraph import dijkstra

    D = net.D
    base_i, snap_km = net.nearest(lat, lon)
    if snap_km > MAX_SNAP_KM:
        raise ValueError("No road found near that point. Please click inside Surabaya.")

    base_id = net.ids[base_i]
    by, bx = net.lat[base_i], net.lon[base_i]

    # Select schools within straight-line radius
    groups = {}
    for s in schools:
        if haversine_km(by, bx, s["lat"], s["lon"]) <= radius_km:
            groups.setdefault(s["node"], []).append(s)

    at_hub = groups.pop(base_i, [])
    if not groups and not at_hub:
        raise ValueError("No schools found within that radius. Try adjusting radius or location.")
    if not groups:
        raise ValueError("The only school found is at the depot itself.")

    stops = [base_i] + list(groups.keys())

    # Directed all-pairs shortest paths across stops
    t0 = time.perf_counter()
    dist, pred = dijkstra(net.csr, directed=True, indices=stops, return_predecessors=True)
    d = dist[:, stops]
    matrix_ms = (time.perf_counter() - t0) * 1000.0

    if not np.isfinite(d).all():
        raise ValueError("Some schools cannot be reached on the road digraph.")

    # Run TSP Solver (Prim's Algorithm MST + DFS Tree Shortcut)
    t1 = time.perf_counter()
    tour, info = solve_tsp(d.tolist())
    tsp_ms = (time.perf_counter() - t1) * 1000.0

    # Expand into detailed road path geometries
    order = tour + [0]
    legs, total_m, total_s, cum = [], 0.0, 0.0, 0.0
    for a, b in zip(order[:-1], order[1:]):
        src, dst = stops[a], stops[b]
        path = [dst]
        while path[-1] != src:
            path.append(int(pred[a][path[-1]]))
        path.reverse()

        coords, leg_m, leg_s = [], 0.0, 0.0
        for x, y in zip(path[:-1], path[1:]):
            u, v = net.ids[x], net.ids[y]
            e = D[u][v]
            leg_m += e["length"]
            leg_s += e["length"] / (e["speed"] / 3.6)
            ec = edge_coords(D, u, v, e.get("geometry"))
            coords.extend(ec if not coords else ec[1:])
        total_m += leg_m
        total_s += leg_s
        cum += leg_m
        legs.append({
            "km": leg_m / 1000.0,
            "cum_km": cum / 1000.0,
            "coords": [[round(p, 6), round(q, 6)] for p, q in coords]
        })

    out_stops = []
    for k, a in enumerate(order[1:-1], 1):
        grp = groups[stops[a]]
        out_stops.append({
            "i": k,
            "name": " + ".join(s["name"] for s in grp),
            "lat": grp[0]["lat"],
            "lon": grp[0]["lon"],
            "leg_km": legs[k - 1]["km"],
            "cum_km": legs[k - 1]["cum_km"]
        })

    return {
        "base": {"lat": by, "lon": bx, "snap_m": snap_km * 1000.0},
        "radius_km": radius_km,
        "stops": out_stops,
        "legs": legs,
        "stats": {
            "num_schools": sum(len(g) for g in groups.values()) + len(at_hub),
            "num_stops": len(stops) - 1,
            "total_km": total_m / 1000.0,
            "minutes": total_s / 60.0,
            "duration": fmt_duration(total_s / 60.0),
            "tsp_ms": tsp_ms,
            "matrix_ms": matrix_ms,
            "method": info["method"],
            "optimal": info["optimal"]
        }
    }


# --------------------------------------------------------------------------- #
# Interactive Web App (Flask + Leaflet.js)
# --------------------------------------------------------------------------- #
PAGE_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>SPPG Mealbox Delivery — TSP Simulator</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link href="https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@400;500;600;700&family=JetBrains+Mono:wght@400;600&display=swap" rel="stylesheet">
  <style>
    :root {
      --primary: #0284c7;
      --primary-dark: #0369a1;
      --accent: #f59e0b;
      --bg: #0f172a;
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
      background: #0b1120;
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
    .subtitle { font-size: 12px; color: #94a3b8; margin-top: 3px; }
    label { display: block; font-size: 12px; font-weight: 600; color: var(--text-muted); margin: 12px 0 4px; text-transform: uppercase; letter-spacing: 0.5px; }
    select, input[type=range] { width: 100%; }
    select {
      padding: 9px 12px;
      border-radius: 6px;
      border: 1px solid var(--border);
      background: #f8fafc;
      font-family: inherit;
      font-size: 13px;
      font-weight: 600;
    }
    .btn-solve {
      width: 100%;
      background: #16a34a;
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
    .btn-solve:hover:not(:disabled) { background: #15803d; }
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
    .order-list li:hover { background: #e0f2fe; }
    .order-num {
      display: inline-block;
      width: 22px;
      height: 22px;
      border-radius: 50%;
      background: #e2e8f0;
      color: #334155;
      font-weight: 700;
      font-size: 11px;
      text-align: center;
      line-height: 22px;
      margin-right: 8px;
    }
  </style>
</head>
<body>
  <div id="app">
    <div id="side">
      <div class="header-bar">
        <div class="badge">TASK 1: TSP SIMULATOR</div>
        <h1>SPPG Mealbox Delivery</h1>
        <div class="subtitle">Surabaya Road Network Digraph</div>
      </div>

      <div class="pad">
        <div style="font-size: 13px; line-height: 1.4; color: #475569;">
          Click anywhere in Surabaya on the map to place the <strong>SPPG Delivery Hub</strong>.
        </div>

        <label>Delivery Radius: <span id="lbl-rad" style="color: #0f172a; font-weight: 700;">__DEFR__ km</span></label>
        <input type="range" id="inp-rad" min="__MINR__" max="__MAXR__" step="0.5" value="__DEFR__">
        <div id="lbl-count" style="font-size: 12px; color: var(--text-muted); margin-top: 4px;">0 schools in radius</div>

        <div style="margin: 10px 0 4px; font-size: 11px; color: #0369a1; background: #e0f2fe; padding: 7px 10px; border-radius: 6px; font-weight: 600;">
          Algorithm: Prim's Algorithm MST + DFS (Week 2 Material)
        </div>

        <button id="btn-go" class="btn-solve" disabled>Compute Optimal Tour</button>
        <div id="msg" style="font-size: 12px; margin-top: 8px; color: #64748b; min-height: 16px;"></div>
      </div>

      <div id="stats-box">
        <div class="stat-grid">
          <div class="stat-card">
            <div class="label">Total Distance</div>
            <div id="st-dist" class="val">0.0 km</div>
          </div>
          <div class="stat-card">
            <div class="label">Travel Time</div>
            <div id="st-time" class="val">0 mins</div>
          </div>
          <div class="stat-card">
            <div class="label">Solver Time</div>
            <div id="st-solver" class="val">0.0 ms</div>
          </div>
          <div class="stat-card">
            <div class="label">Optimality</div>
            <div id="st-opt" class="val" style="color: #16a34a;">Proven</div>
          </div>
        </div>
        <div id="st-method" style="font-size: 11px; color: #64748b; margin-top: 8px; font-style: italic;"></div>
      </div>

      <div id="order-head" class="order-header">
        <span>DELIVERY VISITING ORDER</span>
        <a id="btn-export">Export Route TXT</a>
      </div>
      <ul id="order-list" class="order-list"></ul>
    </div>

    <div id="map"></div>
  </div>

  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet-polylinedecorator/1.6.0/leaflet.polylineDecorator.min.js"></script>
  <script>
    const map = L.map('map').setView([__LAT__, __LON__], 12);
    L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
      maxZoom: 19,
      attribution: '&copy; OpenStreetMap contributors'
    }).addTo(map);

    const radInput = document.getElementById('inp-rad');
    const radLabel = document.getElementById('lbl-rad');
    const countLabel = document.getElementById('lbl-count');
    const btnGo = document.getElementById('btn-go');
    const msgEl = document.getElementById('msg');

    let base = null;
    let baseMarker = null;
    let circle = null;
    let lastData = null;

    const schoolLayer = L.layerGroup().addTo(map);
    const routeLayer = L.layerGroup().addTo(map);
    const arrowLayer = L.layerGroup().addTo(map);
    let schoolsList = [];

    function hav(lat1, lon1, lat2, lon2) {
      const R = 6371;
      const dLat = (lat2 - lat1) * Math.PI / 180;
      const dLon = (lon2 - lon1) * Math.PI / 180;
      const a = Math.sin(dLat/2) ** 2 + Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) * Math.sin(dLon/2) ** 2;
      return 2 * R * Math.asin(Math.sqrt(a));
    }

    function refreshRadius() {
      const r = parseFloat(radInput.value);
      radLabel.textContent = r.toFixed(1) + ' km';
      if (!base) return;

      if (circle) map.removeLayer(circle);
      circle = L.circle([base.lat, base.lon], {
        radius: r * 1000,
        color: '#ef4444',
        fillColor: '#ef4444',
        fillOpacity: 0.06,
        weight: 1.5
      }).addTo(map);

      let count = 0;
      schoolsList.forEach(s => {
        const inside = hav(base.lat, base.lon, s.lat, s.lon) <= r;
        if (inside) count++;
        s.marker.setStyle(inside ? { color: '#0284c7', fillColor: '#0284c7', fillOpacity: 0.8, radius: 5 }
                                 : { color: '#94a3b8', fillColor: '#94a3b8', fillOpacity: 0.4, radius: 3 });
      });
      countLabel.textContent = `${count} schools within ${r.toFixed(1)} km`;
    }

    radInput.addEventListener('input', refreshRadius);

    fetch('/api/schools')
      .then(r => r.json())
      .then(list => {
        list.forEach(s => {
          const m = L.circleMarker([s.lat, s.lon], { radius: 3, color: '#94a3b8', fillColor: '#94a3b8', fillOpacity: 0.4, weight: 1 })
            .bindTooltip(s.name, { direction: 'top' })
            .addTo(schoolLayer);
          schoolsList.push({ lat: s.lat, lon: s.lon, name: s.name, marker: m });
        });
        refreshRadius();
      });

    map.on('click', (e) => {
      base = { lat: e.latlng.lat, lon: e.latlng.lng };
      if (baseMarker) map.removeLayer(baseMarker);
      baseMarker = L.marker(e.latlng, {
        icon: L.icon({
          iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-red.png',
          shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
          iconSize: [25, 41],
          iconAnchor: [12, 41],
          popupAnchor: [1, -34],
          shadowSize: [41, 41]
        })
      }).addTo(map).bindPopup('<b>SPPG Central Hub</b>').openPopup();

      btnGo.disabled = false;
      msgEl.textContent = 'SPPG depot placed. Click Compute Optimal Tour.';
      refreshRadius();
    });

    btnGo.addEventListener('click', async () => {
      if (!base) return;
      btnGo.disabled = true;
      msgEl.textContent = 'Calculating shortest directed TSP tour...';

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
        msgEl.textContent = 'Tour computed successfully.';
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

      // Draw route polylines
      const allCoords = [];
      data.legs.forEach((leg, idx) => {
        allCoords.push(...leg.coords);
        const poly = L.polyline(leg.coords, {
          color: '#0284c7',
          weight: 4,
          opacity: 0.85
        }).addTo(routeLayer);
      });

      if (window.L.polylineDecorator) {
        data.legs.forEach(leg => {
          if (leg.coords.length >= 2) {
            arrowLayer.addLayer(L.polylineDecorator(L.polyline(leg.coords), {
              patterns: [{
                offset: '15%',
                repeat: 120,
                symbol: L.Symbol.arrowHead({
                  pixelSize: 8,
                  polygon: false,
                  pathOptions: { stroke: true, weight: 2.5, color: '#f59e0b', opacity: 0.95 }
                })
              }]
            }));
          }
        });
      }

      // Update statistics
      document.getElementById('stats-box').style.display = 'block';
      document.getElementById('order-head').style.display = 'flex';
      document.getElementById('st-dist').textContent = data.stats.total_km.toFixed(2) + ' km';
      document.getElementById('st-time').textContent = data.stats.duration;
      document.getElementById('st-solver').textContent = data.stats.tsp_ms.toFixed(1) + ' ms';
      document.getElementById('st-opt').textContent = data.stats.optimal ? 'Proven' : 'Heuristic';
      document.getElementById('st-opt').style.color = data.stats.optimal ? '#16a34a' : '#d97706';
      document.getElementById('st-method').textContent = 'Engine: ' + data.stats.method;

      // Populate visiting list
      const listEl = document.getElementById('order-list');
      listEl.innerHTML = '';

      // Hub start
      const li0 = document.createElement('li');
      li0.innerHTML = `<span class="order-num" style="background:#ef4444;color:#fff;">0</span><strong>[START] SPPG Depot</strong>`;
      listEl.appendChild(li0);

      data.stops.forEach(s => {
        const li = document.createElement('li');
        li.innerHTML = `<span class="order-num">${s.i}</span><strong>${s.name}</strong><br>` +
                       `<span style="font-size:11px;color:#64748b;">+${s.leg_km.toFixed(2)} km (Total: ${s.cum_km.toFixed(2)} km)</span>`;
        li.addEventListener('click', () => {
          map.setView([s.lat, s.lon], 16);
        });
        listEl.appendChild(li);
      });

      // Hub return
      const liEnd = document.createElement('li');
      liEnd.innerHTML = `<span class="order-num" style="background:#ef4444;color:#fff;">${data.stops.length + 1}</span><strong>[FINISH] Return to SPPG Depot</strong>`;
      listEl.appendChild(liEnd);

      if (allCoords.length > 0) {
        map.fitBounds(L.latLngBounds(allCoords).pad(0.1));
      }
    }

    document.getElementById('btn-export').addEventListener('click', () => {
      if (!lastData) return;
      const st = lastData.stats;
      let txt = `SPPG MEALBOX DELIVERY ROUTE\\nTotal Distance: ${st.total_km.toFixed(2)} km\\nEstimated Time: ${st.duration}\\nSolver Time: ${st.tsp_ms.toFixed(2)} ms\\nMethod: ${st.method}\\n\\n`;
      txt += `0. [START] SPPG Central Depot\\n`;
      lastData.stops.forEach(s => {
        txt += `${s.i}. ${s.name} (+${s.leg_km.toFixed(2)} km, Cum: ${s.cum_km.toFixed(2)} km)\\n`;
      });
      txt += `${lastData.stops.length + 1}. [FINISH] Return to SPPG Depot\\n`;

      const blob = new Blob([txt], { type: 'text/plain;charset=utf-8' });
      const a = document.createElement('a');
      a.href = URL.createObjectURL(blob);
      a.download = 'sppg_delivery_route.txt';
      a.click();
    });
  </script>
</body>
</html>
"""


def create_app(net, schools):
    from flask import Flask, Response, jsonify, request

    # Snap schools to nearest road nodes on start
    for s in schools:
        s["node"], _ = net.nearest(s["lat"], s["lon"])

    app = Flask(__name__)
    lock = threading.Lock()
    public_schools = [{"name": s["name"], "lat": s["lat"], "lon": s["lon"]} for s in schools]

    html_content = (PAGE_HTML
                    .replace("__LAT__", str(DEFAULT_CENTER[0]))
                    .replace("__LON__", str(DEFAULT_CENTER[1]))
                    .replace("__MINR__", str(MIN_RADIUS_KM))
                    .replace("__MAXR__", str(MAX_RADIUS_KM))
                    .replace("__DEFR__", str(DEFAULT_RADIUS_KM)))

    @app.get("/")
    def index():
        return Response(html_content, mimetype="text/html")

    @app.get("/api/schools")
    def get_schools():
        return jsonify(public_schools)

    @app.post("/api/solve")
    def solve():
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
                res = run_tsp(net, schools, lat, lon, radius)
            return jsonify(res)
        except ValueError as e:
            return jsonify(error=str(e)), 422

    return app


def main():
    G = load_surabaya_graph()
    D = build_digraph(G)
    schools = load_schools()
    app = create_app(RoadNet(D), schools)
    url = f"http://127.0.0.1:{PORT}"
    print(f"[+] Server started. Open {url}")
    threading.Timer(1.2, lambda: webbrowser.open(url)).start()
    app.run(host="127.0.0.1", port=PORT, debug=False, threaded=True)


if __name__ == "__main__":
    main()
