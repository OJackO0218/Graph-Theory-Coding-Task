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
| # | Task | Algorithm Used | Description |
|---|------|----------------|-------------|
| 1 | TSP – SPPG Simulator | Nearest-Neighbor + Dijkstra | Find efficient school mealbox delivery route in Surabaya |
| 2 | CPP – DLH Simulator | Odd-Node Matching + Hierholzer | Find shortest closed route covering all streets in Surabaya |
| 3 | Knight's Tour | Warnsdorff's Heuristic | Simulate knight visiting every chessboard square exactly once |

---

## Prerequisites

```bash
pip install osmnx networkx folium
```
- Python 3.8+
- Task 3 uses only standard library (no external packages required).

---

## How to Run

### Task 1 — TSP (SPPG Simulator)
```bash
cd task1_tsp
python tsp.py
```
- Fetches real road network and schools in Surabaya via OpenStreetMap (OSMnx).
- Includes automatic offline fallback so the simulator never fails if public Overpass servers are congested.
- Configurable parameters at the top of `tsp.py`: `SPPG_LAT`, `SPPG_LON`, `RADIUS_KM`.
- Output: Terminal stats + interactive route map saved as `tsp_map.html`.

**Sample terminal output:**
```text
Loading Surabaya road network from OpenStreetMap...
Loaded 6 schools within 3 km.

Solving TSP...
--- Results ---------------------------------
  Schools visited    : 6
  Total distance     : 5.08 km
  Est. travel time   : 10.2 minutes
  Solver time        : 0.001 seconds

  Visiting order:
    1. SPPG (start)
    2. SMA Negeri 1 Surabaya
    3. SMP Negeri 1 Surabaya
    4. SMA Negeri 2 Surabaya
    5. SMA Negeri 5 Surabaya
    6. SMP Negeri 3 Surabaya
    7. SD Negeri Kaliasin I
    -> Return to SPPG

Map saved -> tsp_map.html
```

---

### Task 2 - CPP (DLH Simulator)
```bash
cd task2_cpp
python cpp.py
```
- Finds the shortest closed Eulerian route traversing all designated streets at least once.
- Detects odd-degree intersections, balances them by duplicating shortest paths, and finds the circuit using Hierholzer's algorithm.
- Configurable parameters at the top of `cpp.py`: `BASE_LAT`, `BASE_LON`, `RADIUS_KM`.
- Output: Terminal stats + interactive route map saved as `cpp_map.html`.

**Sample terminal output:**
```text
Loading Surabaya road network from OpenStreetMap...
Graph: 6 nodes, 18 edges
Odd-degree nodes: 6
Solving CPP...
--- Results ---------------------------------
  Edges (roads) covered : 18
  Circuit steps         : 12
  Total distance        : 6.07 km
  Est. travel time      : 18.2 minutes
  Solver time           : 0.007 seconds

Map saved -> cpp_map.html
```

---

### Task 3 — Knight's Tour
```bash
cd task3_knights_tour
python knights_tour.py
```

**Input format:**
- Line 1: `cols rows` (e.g. `5 5`)
- Line 2: `start_col start_row` (e.g. `2 2`, 1-indexed)

**Sample run:**
```text
Enter board size (cols rows): 5 5
Enter start position (col row): 2 2

Board: 5 x 5, Start: (2, 2)
Solving...
Path found! (25 moves)

Coordinates visited (col, row):
    1. (2, 2)
    2. (4, 1)
    3. (5, 3)
    ...
   25. (1, 5)

 23 16  7  2 21
  6  1 22 15  8
 17 24 11 20  3
 12  5 18  9 14
 25 10 13  4 19
```
If no solution exists, outputs: `path not found`.

---

## Interactive Visual HTML Files
Open directly in any web browser for interactive visualization:
- `task1_tsp/tsp_visual.html` — Animated delivery truck route representation
- `task2_cpp/cpp_visual.html` — Graph illustration of odd-degree nodes & duplicated edges
- `task3_knights_tour/knights_tour_visual.html` — Fully interactive animated chessboard solver

---

## Algorithm Documentation Report
See [`docs/algorithm_explanation.pdf`](docs/algorithm_explanation.pdf) (or editable [`docs/algorithm_explanation.docx`](docs/algorithm_explanation.docx)) for the complete report explaining the algorithm ideas, implementation details, complexity analysis, and visual representations.

## Algorithm Documentation
See [`docs/algorithm_explanation.docx`](docs/algorithm_explanation.docx) for the full algorithm explanation and complexity analysis report.
