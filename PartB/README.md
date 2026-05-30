# Part B — Traffic Flow Prediction (TBRGS)

Traffic-based route guidance using LSTM and GRU models trained on SCATS October 2006 data.

---
## Setup

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Prepare the datasets

Place `Scats Data October 2006.xls` in the `data/` directory, then run:

```bash
cd PartB/data
python process_df.py
```

This reads the XLS and outputs per-direction `train.csv` / `test.csv` splits under `data/SCATS_Data/`.

### 3. Download trained models

Download the zipped `trained_models` file, place it in the `PartB/` directory, and unzip it:

```bash
unzip trained_models.zip
```

---

## Running

### Route Visualiser (GUI)

```bash
cd PartB
python visualiser.py
```

Select origin, destination, departure time, algorithm, and model from the dropdowns then click **Find Routes**. Routes are drawn on the network canvas and listed by travel time (best to worst). Click a route in the list to highlight it.

### Command-line routing

```bash
cd PartB
python pathFinder.py
```

Prints the top-5 routes for the hardcoded origin/destination in `main()`. Edit that function to change parameters.

### Training

```bash
cd PartB
python train.py
```

### Testing

```bash
cd PartB
python test.py
```

Evaluates a single SCATS site and direction, prints metrics (MAE, RMSE, MAPE, R²), and shows a prediction plot. `evaluate_all()` computes averages across all trained directions.

---

## Search Algorithms

All algorithms are in `pathFinder.py` and accept the same interface. Pass via the `method` parameter of `find_routes()`.

| Key | Algorithm |
|-----|-----------|
| `AS` | A* (default) |
| `BFS` | Breadth-First Search |
| `DFS` | Depth-First Search |
| `GBFS` | Greedy Best-First Search |
| `DLS` | Depth-Limited Search (CUS1) |
| `ALT` | A* with Landmark Triangle Inequality (CUS2) |

Routes are found using Top k shortest paths with the selected algorithm. Each edge's travel time is predicted at the actual arrival time at that node.

---

## Modules

| File | Description |
|------|-------------|
| `pathFinder.py` | Search algorithms, Yen's k-shortest, `find_routes()` API |
| `visualiser.py` | Tkinter GUI — network canvas and route highlighting |
| `predict.py` | ML inference — predicts traffic flow for a SCATS site and November datetime |
| `models.py` | Keras model definitions (LSTM, GRU, custom CNN-RNN) |
| `train.py` | Training script |
| `test.py` | Evaluation and plotting |
| `graph/graph.py` | Builds the road network from `road_data.csv` |
| `graph/traveltime.py` | Converts predicted flow to speed and travel time (seconds) |
| `graph/search_node.py` | `SearchNode` used during pathfinding; `build_path()` to trace routes |
| `graph/node.py` | Physical `Node` — SCATS number, coordinates, road names |
| `graph/edge.py` | `Edge` between two nodes — road name and distance (km) |
| `graph/parse_road_data.py` | Regenerates `road_data.csv` from the VicRoads XLS |
| `data/process_df.py` | Preprocesses SCATS XLS into per-direction train/test CSVs |

---

## Graph regeneration (optional)

Only needed if `road_data.csv` needs to be rebuilt from scratch. Requires `SCATSSiteListingSpreadsheet_VicRoads.xls` placed at `graph/site_road_data/`.

```bash
cd PartB/graph
python parse_road_data.py
python parse_site_type.py
```
