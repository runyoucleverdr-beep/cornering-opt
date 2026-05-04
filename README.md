# Modeling Information Diffusion on Twitter Using Breadth-First Search  
### A Case Study of the SNAP Higgs Twitter Dataset

---

## 1. Project Overview

This project studies how scientific news spreads through a large social network using the **SNAP Higgs Twitter Dataset**. The main goal is to model the Twitter retweet network as a directed graph and apply **Breadth-First Search (BFS)** to analyze information diffusion.

The project focuses on three main questions:

1. **How far** can information spread in the retweet network?
2. **How deep** can a diffusion cascade extend in terms of hop count?
3. **Which early users** are structurally positioned to trigger broader cascades?

The current implementation uses the **retweet layer** of the Higgs dataset and reverses the edge direction to better represent **information flow** from source users to later retweeters.

---

## 2. Research Motivation

Social media platforms are a major channel for the spread of public information. When an important scientific event occurs, information does not diffuse uniformly across the network. Some users trigger broad cascades, while others remain in small local structures.

This project uses graph algorithms to measure that structural difference. Rather than treating diffusion as an abstract concept, it quantifies it using:

- reachable audience size
- propagation depth
- shortest hop distance

---

## 3. Dataset

This project uses the **SNAP Higgs Twitter Dataset**, which was built around Twitter activity related to the Higgs boson discovery.

### Main files used
- `higgs-retweet_network.edgelist.gz`
- `higgs-activity_time.txt.gz`

### Data description
- The retweet network is modeled as a **directed graph**
- Each node represents a Twitter user
- Each edge represents a retweet relation
- The activity file provides:
  - `user_a`
  - `user_b`
  - `timestamp`
  - `interaction`

The current project uses:

- the **retweet network** for BFS traversal
- the **activity file** to identify candidate source users

### Important note on edge direction
To analyze **information diffusion**, this project reverses the original retweet edge direction. This is because the observed retweet action and the direction of information flow are not the same.

---

## 4. Method

### Core algorithm
The main algorithm used in this project is **Breadth-First Search (BFS)**.

### Why BFS
BFS is appropriate because it explores the graph level by level, which naturally matches the layered structure of information diffusion.

### Metrics currently computed
For each selected source user, the project computes:

- **reachable_nodes**: number of users reachable from the source
- **reachable_ratio**: reachable nodes divided by total graph size
- **max_depth**: maximum BFS hop level
- **avg_distance**: average shortest-path distance to reachable nodes
- **median_distance**: median shortest-path distance

### Current source strategy
The current experiment selects **early original-source candidates** from the retweet activity data, rather than early retweeters. This design better matches the reversed information-flow graph.

---

## 5. Repository Structure

```text
higgs-diffusion-bfs/
├─ README.md
├─ requirements.txt
├─ .gitignore
├─ config/
│  └─ default.yaml
├─ data/
│  ├─ raw/
│  ├─ interim/
│  └─ processed/
├─ outputs/
│  ├─ figures/
│  ├─ tables/
│  └─ logs/
├─ notebooks/
│  ├─ 01_data_check.ipynb
│  ├─ 02_graph_eda.ipynb
│  └─ 03_bfs_results.ipynb
├─ src/
│  ├─ __init__.py
│  ├─ config.py
│  ├─ io_utils.py
│  ├─ graph_builder.py
│  ├─ activity_parser.py
│  ├─ source_selection.py
│  ├─ bfs_analysis.py
│  ├─ metrics.py
│  ├─ visualize.py
│  └─ pipeline.py
├─ scripts/
│  ├─ __init__.py
│  ├─ run_data_check.py
│  ├─ run_bfs_experiment.py
│  └─ run_full_pipeline.py
└─ tests/
   ├─ test_graph_builder.py
   ├─ test_bfs_analysis.py
   └─ test_source_selection.py
````

---

## 6. Environment Setup

### Recommended Python version

This project is best run with **Python 3.10**.

### Create environment with Conda

```bash
conda create -n higgs-bfs python=3.10 -y
conda activate higgs-bfs
pip install -r requirements.txt
```

---

## 7. Download the Dataset

The raw dataset files are **not included** in this repository.
Please download them manually and place them in:

```text
data/raw/
```

### Required files

```text
data/raw/higgs-retweet_network.edgelist.gz
data/raw/higgs-activity_time.txt.gz
```

### Example PowerShell download commands

```powershell
New-Item -ItemType Directory -Path .\data\raw -Force | Out-Null

Invoke-WebRequest `
  -Uri "https://snap.stanford.edu/data/higgs-retweet_network.edgelist.gz" `
  -OutFile ".\data\raw\higgs-retweet_network.edgelist.gz"

Invoke-WebRequest `
  -Uri "https://snap.stanford.edu/data/higgs-activity_time.txt.gz" `
  -OutFile ".\data\raw\higgs-activity_time.txt.gz"
```

---

## 8. Configuration

The main configuration file is:

```text
config/default.yaml
```

Example:

```yaml
paths:
  retweet_network: data/raw/higgs-retweet_network.edgelist.gz
  activity_file: data/raw/higgs-activity_time.txt.gz
  output_tables: outputs/tables
  output_figures: outputs/figures

experiment:
  num_early_users: 10
  num_random_users: 10
  random_seed: 42

graph:
  reverse_for_information_flow: true

plot:
  top_k_sources: 10
```

---

## 9. How to Run

### Step 1: Data check

This verifies that the retweet network and activity file can be loaded correctly.

```bash
python scripts/run_data_check.py
```

### Step 2: Run BFS experiment

This runs the current diffusion analysis pipeline.

```bash
python scripts/run_bfs_experiment.py
```

### Step 3: Full pipeline

Currently equivalent to the BFS experiment script.

```bash
python scripts/run_full_pipeline.py
```

---

## 10. Outputs

After running the BFS experiment, the following files are generated:

### Tables

* `outputs/tables/bfs_source_summary.csv`

### Figures

* `outputs/figures/top_sources_reach.png`
* `outputs/figures/top_sources_depth.png`
* `outputs/figures/representative_bfs_levels.png`

---

## 11. Current Progress

The current version of the project already supports:

* loading the Higgs retweet graph
* reversing edge direction for information-flow analysis
* parsing the activity file
* selecting early source candidates
* running BFS from multiple source nodes
* computing diffusion metrics
* exporting tables and figures

Initial results show that diffusion reach differs dramatically across source users. Some source nodes can reach only a few users, while others can reach tens of thousands of nodes. This suggests that **structural position in the graph strongly affects diffusion potential**.

---

## 12. Current Interpretation

The first meaningful experiment shows that:

* not all early users trigger large diffusion cascades
* structurally favorable users can reach a very large portion of the retweet graph
* several source users appear to belong to the same large reachable diffusion region

This supports the central project claim that **diffusion is shaped not only by timing, but also by graph structure**.

---

## 13. Planned Next Steps

The next development stage will add:

* comparison across different source-selection strategies

  * earliest original sources
  * top out-degree users
  * random users
* stronger baseline analysis
* more detailed graph diagnostics
* improved visualizations for report writing

Possible future extensions include:

* time-window analysis
* comparison across retweet / reply / mention layers
* runtime analysis across different source sets

---

## 14. Notes

* Raw data files are excluded from version control.
* Generated figures and tables are also excluded from Git.
* The current BFS analysis is **unweighted**. If edge weights are incorporated later, the project may be extended to weighted diffusion analysis.

---

## 15. License / Academic Use

This repository is intended for academic course-project use.
Please check the original dataset source for data usage conditions.

````
