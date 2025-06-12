# Multicolour-Block Robosuite Environment

This repo contains:

* **robostack/** – your custom code  
  * `envs/multi_color_block_env.py`  ← the new environment  
  * `setup_env.py`                   ← quick manual test (random actions)
* **robosuite/** – *vendored* copy of Robosuite, with a one-line back-compat
  patch that re-exports `XMLError`.
* standard Robosuite folders (`docs/`, `tests/`, etc.) plus project files
  (`pyproject.toml`, `requirements*.txt`, …).

Clone, create a fresh env, install the local packages, and you’re good to go.

---

## 1.  Prerequisites

* Python ≥ 3.9 (tested on 3.10)  
* MuJoCo 2.3 + mujoco-python (pulled automatically by `pip install robosuite`)
* Conda or *virtualenv* recommended

---

## 2.  Quick setup

```bash
# clone the repo
git clone <your-repo-url>
cd project       # top-level folder that contains robostack/ and robosuite/

# 1️⃣  create + activate an isolated environment
conda create -n robosim-arm python=3.10
conda activate robosim-arm            # or use venv

# 2️⃣  install Python dependencies
pip install -r requirements.txt       # if you maintain one
# install *local* Robosuite in editable mode so our patch is used
pip install -e robosuite
# (optional) install robostack itself as a package
pip install -e robostack

# 3️⃣  one-time macro generation (silences startup warnings)
python robosuite/robosuite/scripts/setup_macros.py

# 4️⃣  run the demo
python robostack/setup_env.py
```

You should see a window similar to the screenshot in which a Panda arm
executes random motions while five coloured blocks spawn at random
collision-free poses on the table.

---

## 3.  Repository layout
