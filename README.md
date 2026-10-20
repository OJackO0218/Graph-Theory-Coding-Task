# Graph Theory Coding Task
**Institut Teknologi Sepuluh Nopember**

## Team Identity
|    NRP     |           Nama             |
| :--------: |       :------------:       |
| 5025251260 | Aqilah Ibrahim             |
| 5025251161 | Rizqi Arya Kuskhilbyano    |
| 5025251009 | Athar Rozy Rasyidan                    |
| 5025251017 | Wafi Fawwaz Sutisna                    |


## Project Overview

This repository contains state-of-the-art graph-theoretic solutions and interactive visual simulators for the three core problems assigned in the ITS Graph Theory Coding Task:

1. **Task 1: Travelling Salesman Problem (SPPG Mealbox Delivery)**  
   Simulates optimal meal delivery from a central SPPG (Satuan Pelayanan Pemenuhan Gizi) to all public schools within a designated radius in Surabaya, navigating the real OpenStreetMap directed road graph.
2. **Task 2: Chinese Postman Problem (DLH Garbage Collection)**  
   Simulates full street coverage of all arterial roads (trunk, primary, secondary, and tertiary) by Dinas Lingkungan Hidup (DLH) dump trucks using a directed Rural Postman formulation on Surabaya's road network.
3. **Task 3: The Knight's Tour (Dynamic Chessboard Simulation)**  
   Solves the Hamiltonian path problem for a chess knight on arbitrary $W \times H$ chessboards from any starting coordinate $(x, y)$, complete with instant mathematical impossibility pruning and step-by-step animation.

---

## Algorithmic Innovations (Distinct Implementations)

Unlike naive or standard nearest-neighbor/linear-relaxation implementations, this project introduces mathematically rigorous, distinct graph algorithms:

| Task | Traditional / Group 4 Baseline | **Our Implementation (Single Course Algorithm)** | Theoretical Alignment & Material Reference |
| :--- | :--- | :--- | :--- |
| **Task 1: TSP** | Nearest Neighbor + Or-Opt + MILP | **Prim's Algorithm MST (Week 2, Slide 18) + Depth-First Search (DFS) Tree Traversal Shortcut (Week 2, Slide 13)** | The classic 2-approximation metric TSP theorem from Graph Theory using Prim's MST and DFS preorder shortcutting, refined by directed 2-Opt. |
| **Task 2: CPP** | Multi-source Dijkstra greedy bridge + NetworkX flow + MILP | **Hierholzer's Algorithm (Week 3, Slides 13–15) + Eulerian Digraph Balancing (Week 3, Slide 9)** | Enforces the Eulerian condition ($d_{\text{in}} = d_{\text{out}}$) via bipartite matching, then traces the complete Eulerian circuit in $O(\|E\|)$ linear time using Hierholzer's algorithm. |
| **Task 3: Knight's Tour** | Warnsdorff with degree-of-degree tie breaking | **Depth-First Search (DFS) Backtracking (Week 2, Slide 13 / Week 4, Slide 5) + Bipartite Parity Pruning** | Formulates knight movement as a Hamiltonian Path search on chessboard graphs, using stack-based DFS Backtracking with lookahead and $O(1)$ parity checks. |

---

## Prerequisites & Setup

### Environment Requirements
* **Python 3.8+** (Tested on Python 3.10 – 3.13)
* Standard libraries: `math`, `time`, `json`, `threading`, `webbrowser`, `http.server`
* Graph & Geographic Data Libraries:
  ```bash
  python -m pip install osmnx networkx numpy scipy flask shapely
  ```

---

## How to Run

### Task 1: Travelling Salesman Problem (SPPG Simulator)
* **Algorithm Used:** **Prim's Algorithm MST + DFS Tree Shortcut** (Week 2, Slides 13 & 18)
1. Launch the application:
   ```bash
   python tsp.py
   ```
2. The interactive web application will open automatically at `http://127.0.0.1:5001`.
3. **Usage:**
   * Click any location on the Surabaya map to establish the **SPPG Delivery Hub**.
   * Adjust the delivery radius slider ($0.5\text{ km} - 10.0\text{ km}$).
   * Click **Compute Optimal Tour** to see the route, visiting sequence, driving duration, and solver runtime.

---

### Task 2: Chinese Postman Problem (DLH Dump Truck Simulator)
* **Algorithm Used:** **Hierholzer's Algorithm** (Week 3, Slides 13–15)
1. Launch the application:
   ```bash
   python cpp.py
   ```
2. The interactive web application will open automatically at `http://127.0.0.1:5002`.
3. **Usage:**
   * Click anywhere in Surabaya to set the **DLH Dump Truck Depot**.
   * Set the collection radius slider ($0.3\text{ km} - 5.0\text{ km}$).
   * Click **Compute Collection Route**.
   * View collection segments (vibrant green) vs deadheading/transit segments (dashed orange), street traversal order, and total distance.

---

### Task 3: Knight's Tour Simulator
* **Algorithm Used:** **Depth-First Search (DFS) Backtracking** (Week 2, Slide 13 / Week 4, Slide 5)
The Knight's Tour simulator supports both **Standard Console Mode** and **Interactive Visual Mode**:

#### Console Mode (Task Specification I/O)
Run directly in your terminal:
```bash
python chessknight.py
```
Input four space- or newline-separated integers:
```
<width> <height>
<start_x> <start_y>
```
* If a tour exists, the program prints the coordinates in visiting order:
  ```
  x0,y0
  x1,y1
  ...
  ```
* If impossible, the program prints:
  ```
  path not found
  ```

#### Interactive Visual Mode
Launch the animated HTML5 Canvas interface:
```bash
python chessknight.py --visual
```
Opens `http://127.0.0.1:5003` with board configuration, step-by-step playback, speed slider, path tracing, and live statistics.

---

## Sample Test Cases & Verification

### Test Case 1: Knight's Tour - $5 \times 5$ Board from $(2, 2)$
* **Input:**
  ```
  5 5
  2 2
  ```
* **Output:**
  ```
  2,2
  0,1
  1,3
  3,4
  4,2
  ... (25 visited coordinates)
  ```
* **Status:** `Path Found` (Total: 25 steps, runtime < 1 ms).

### Test Case 2: Knight's Tour - $4 \times 4$ Board (Proven Impossibility)
* **Input:**
  ```
  4 4
  0 0
  ```
* **Output:**
  ```
  path not found
  ```
* **Theoretical Justification:** By Schwenk's Theorem (1991), no knight's tour exists on boards with dimension $4 \times 4$ because corner and edge squares cannot form a connected Hamiltonian cycle. Pruned in $O(1)$ time.

### Test Case 3: Knight's Tour - Odd-Parity Violation
* **Input:**
  ```
  5 5
  0 1
  ```
* **Output:**
  ```
  path not found
  ```
* **Theoretical Justification:** A chessboard is a bipartite graph. With $25$ squares (odd), an open tour must start on the majority color (where $x + y$ is even). Since $(0 + 1) \pmod 2 = 1$, a full tour is mathematically impossible. Pruned in $O(1)$ time.

---
