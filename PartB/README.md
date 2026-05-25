# Part B — Traffic Flow Prediction (TBRGS)

Traffic-based route guidance using LSTM and GRU models trained on SCATS October 2006 data.

---
## Setup

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Prepare the datasets

Navigate to the `data/` directory and run the processing script:

> This reads `Scats Data October 2006.xls` and outputs the `SCATS_Data/` folder containing all required train/test splits.
>Upload the `Scats Data October 2006.xls` in the data folder.

```bash
cd PartB/data
python process_df.py
```

### 3. Download trained models

Download the zipped `trained_models` file, place it in the `PartB/` directory, and unzip it:

```bash
# From PartB/
unzip trained_models.zip
```

> The models were trained using `train.py` with `SCATS_Data` and `trained_models/model.py`.

---

## Usage

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

- Tests an individual SCATS site and direction — configure the site number and direction inside `main()`.
- Outputs evaluation metrics and a plot.
- `evaluate_all()` computes average metrics across both models. Close the graph window and wait for few minutes. Warnings can be ignored.

### Saving Predictions

```bash
cd PartB
python predict_save.py
```

> Computes and saves predictions for the entire October 2006 timeframe and saves them as CSV files under `predictions/`. This may take a few minutes.

---

## Graph & Travel Time

> **Note:** The `SCATSSiteListingSpreadsheet_VicRoads.xls` must be placed at `graph/site_road_data/SCATSSiteListingSpreadsheet_VicRoads.xls` before running the parse scripts.

```bash
cd PartB/graph
python parse_road_data.py     # Parses VicRoads 
python parse_site_type.py     # Confirms all SCATS sites are intersections (INT)
```

| Module | Description |
|---|---|
| `graph.py` | Builds the road network graph from nodes and edges |
| `traveltime.py` | Converts traffic volume to speed and calculates travel time |
| `load_prediction.py` | Returns model predictions for a given SCAT and time of day |

