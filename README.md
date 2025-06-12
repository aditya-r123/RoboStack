# Multicolour-Block Robosuite Environment

This repository provides a minimal custom task for **Robosuite** that spawns
multiple coloured blocks on a table in front of a Panda arm.  
Everything task-specific lives in the `robostack/` package; Robosuite itself is
pulled from PyPI.

---

## 1.  Prerequisites
* Python ≥ 3.9 (tested with 3.10)
* MuJoCo ≥ 2.3 (installed automatically by `pip install robosuite`)
* Conda or **virtualenv** recommended

---

## 2.  Quick setup

```bash
# 0. clone the repo
git clone <your-repo-url>
cd <repo>                      # root folder that contains robostack/

# 1. create + activate an isolated environment
conda create -n robosim-arm python=3.10
conda activate robosim-arm          # or use venv

# 2. install Python dependencies
pip install -r requirements.txt     # if you maintain one
# Robosuite 1.4.0 keeps the manipulation-task modules our env depends on
pip install "robosuite==1.4.0"
# (optional) install robostack itself as editable package
pip install -e robostack

# 3. one-time macro generation (silences startup warnings)
python -m robosuite.scripts.setup_macros

# 4. run the demo
python robostack/setup_env.py
```

You should see a window where the Panda arm performs random motions while
five coloured blocks appear at random, collision-free poses.

---

## 3.  Repository layout
