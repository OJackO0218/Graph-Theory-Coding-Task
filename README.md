# Graph Theory Coding Task — Week 6-8
**Institut Teknologi Sepuluh Nopember**

## Team Identity
|    NRP     |           Nama             |
| :--------: |       :------------:       |
| 5025251260 | Aqilah Ibrahim             |
| 5025251161 | Rizqi Arya Kuskhilbyano    |
| 5025251009 | Athar Rozy Rasyidan                    |
| 5025251017 | Wafi Fawwaz Sutisna                    |

---

## Tasks Overview
| # | Task | Description |
|---|------|-------------|
| 1 | TSP – SPPG Simulator | Find efficient school delivery route in Surabaya |
| 2 | CPP – DLH Simulator | Find efficient garbage collection route in Surabaya |
| 3 | Knight's Tour | Simulate knight visiting every chessboard square once |

---

## Prerequisites

```bash
pip install osmnx networkx folium
```

Python 3.8+

---

## How to Run

### Task 1 — TSP (SPPG Simulator)
```bash
cd task1_tsp
python tsp.py
```
- Opens with default center (Surabaya) and 3 km radius
- To customize: edit `SPPG_LAT`, `SPPG_LON`, `RADIUS_KM` at the top of `tsp.py`
- Output: `tsp_map.html` (open in browser), printed route details in terminal

**Sample output:**
```
Loading Surabaya road network...
Found 12 schools within 3 km.
Solving TSP...
── Results ─────────────────────────────────
  Schools visited    : 12
  Total distance     : 18.4 km
  Est. travel time   : 36.8 minutes
  Solver time        : 2.341 seconds
```

---

### Task 2 — CPP (DLH Simulator)
```bash
cd task2_cpp
python cpp.py
```
- Default: Surabaya center, 1 km radius (larger radius = longer runtime)
- To customize: edit `BASE_LAT`, `BASE_LON`, `RADIUS_KM` at the top of `cpp.py`
- Output: `cpp_map.html` (open in browser), circuit details in terminal

**Sample output:**
```
Loading Surabaya road network...
Graph: 142 nodes, 310 edges
Odd-degree nodes: 24
Solving CPP...
── Results ─────────────────────────────────
  Edges (roads) covered : 310
  Total distance        : 12.7 km
  Est. travel time      : 38.1 minutes
  Solver time           : 4.512 seconds
```

---

### Task 3 — Knight's Tour
```bash
cd task3_knights_tour
python knights_tour.py
```

**Sample input/output:**
```
Enter board size (cols rows): 5 5
Enter start position (col row): 2 2

Path found! (25 moves)
  1. (2, 2)
  2. (4, 1)
  3. (5, 3)
  ...
 25. (3, 4)
```

If no tour exists:
```
path not found
```

---

## Visual HTML Files
Each task has an HTML visual explainer (no server needed, just open in browser):
- `task1_tsp/tsp_visual.html`
- `task2_cpp/cpp_visual.html`
- `task3_knights_tour/knights_tour_visual.html` ← fully interactive!

---

## Algorithm Documentation
See [`docs/algorithm_explanation.docx`](docs/algorithm_explanation.docx) for the full algorithm explanation and complexity analysis report.
