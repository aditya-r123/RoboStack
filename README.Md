# RoboStack


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

```
RoboStack/
├── controllers/
│   └── policies.py       # Contains high-level policies for the robot
├── envs/
│   └── multi_color_block_env.py # Defines the custom Robosuite environment
├── README.Md             # This file
└── setup_env.py          # Script to set up and run the environment
```

---

## 4. Project Overview
This project focuses on developing a sophisticated robotic manipulation policy for a Panda arm in the Robosuite simulation environment. The primary goal is to enable the robot to autonomously stack blocks of the same color into stable towers. The environment, defined in `envs/multi_color_block_env.py`, spawns multiple blocks of various colors at random locations on a table. The core logic, located in `controllers/policies.py`, implements a state-driven policy to identify, plan, and execute the stacking tasks.

---

## 5. Stacking Policy (`MultiColorStackPolicy`)
The key to our solution is the `MultiColorStackPolicy`, a robust, state-based policy designed to handle the entire stacking process from start to finish.

### Planning
Before taking any action, the policy first builds a plan by:
1.  Observing the colors and positions of all blocks on the table.
2.  Grouping the blocks by color.
3.  For each color with multiple blocks, it designates one block as the base of a stack.
4.  It then creates a sequence of tasks to pick up the other blocks of that color and place them on the corresponding base.

### Execution
The policy executes this plan using a state machine that guides the robot through the necessary steps for each block:
- **Approach**: Hover above the target block.
- **Grasp**: Descend, close the gripper, and lift the block vertically to a safe height.
- **Reposition**: Move the grasped block above the target stack, ensuring the path is clear of any existing stacks to prevent collisions.
- **Stack**: Carefully lower the block onto the stack.
- **Retreat**: Open the gripper and move the arm up and away, ready for the next task.

The policy includes recovery mechanisms to handle situations where it gets stuck and can even re-plan if it detects that a block has fallen off a stack.

---

## 6. Project Milestones
We have achieved the following milestones in this project:

### 1. Detect Different Colors of Blocks
The simulation environment is configured to provide the policy with perfect information about block colors and positions. `MultiColorBlockEnv` attaches a list of block colors to the observations, which the `MultiColorStackPolicy` uses during its planning phase to group blocks.

### 2. Differentiate Same-Colored Blocks and Choose a Base
The `_build_plan` method within `MultiColorStackPolicy` is responsible for this. It iterates through all blocks, groups them by color, and for each color group, selects the first block it encounters as the foundation for the stack. The remaining blocks in that color group are then scheduled to be stacked on top of it.

### 3. Safe Vertical Lifting and Repositioning
This is a core feature of our policy's state machine, as implemented in `controllers/policies.py`.
- **Vertical Lift**: After grasping a block (State 2), the policy raises the arm straight up to a predefined `lift_h`.
- **Collision-Free Movement**: Before moving to the next block, the arm moves to a calculated `safe_z` height (State 7), which is guaranteed to be above the highest block or stack on the table. This ensures the arm never collides with existing stacks while traversing the workspace.

### 4. Precise and Collision-Free Stacking
As implemented in `controllers/policies.py`, the policy ensures precise stacking. After moving the block above the target stack (State 3), it lowers it straight down (State 4), minimizing lateral movement that could knock over the stack. This precise motion control, governed by a PID controller, ensures that blocks are placed gently and accurately.
