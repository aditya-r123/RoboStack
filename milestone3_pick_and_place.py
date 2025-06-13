"""
Milestone 3  –  fast pick-and-place demo
---------------------------------------

• picks one non-base block
• lifts straight up, translates at a safe hover height
• stacks it on its colour's base block
• uses OSC-Pose controller's set_goal (no slow incremental loop)
"""

import time
import collections
import numpy as np
import robosuite, envs


# ---------------------------------------------------------------- helpers
def goto(env, goal_pos, goal_quat, gripper, tol=2e-3, max_steps=400):
    """
    Drive end–effector to (pos, quat); keep sending the same pose goal until
    either the tool is within @tol metres OR @max_steps sim steps have passed.
    """
    robot = env.robots[0]
    robot.controller.set_goal(np.concatenate([goal_pos, goal_quat]))

    action = np.zeros(env.action_spec[0].shape)
    if robot.gripper:
        action[-1] = gripper

    steps = 0
    while steps < max_steps:
        env.step(action)
        env.render()

        if np.linalg.norm(env._eef_xpos - goal_pos) < tol:
            break
        steps += 1


# ---------------------------------------------------------------- build env
env = robosuite.make(
    "MultiColorBlockEnv",
    robots="Panda",
    has_renderer=True,
    has_offscreen_renderer=True,
    use_camera_obs=False,
    horizon=2000,
    ignore_done=True,            # keep stepping as long as we like
    control_freq=20,
)

obs   = env.reset()
robot = env.robots[0]

# ---------------------------------------------------------------- collect blocks + choose one colour to stack
Block = collections.namedtuple("Block", "name rgba pos")

blocks = [
    Block(
        blk.name,
        tuple(blk.rgba),
        env.sim.data.body_xpos[env.sim.model.body_name2id(blk.root_body)].copy(),
    )
    for blk in env.blocks
]

groups = collections.defaultdict(list)
for b in blocks:
    groups[b.rgba].append(b)

# pick a colour that has at least two cubes (one base + one mover)
colour = next(k for k, v in groups.items() if len(v) > 1)
base   = min(groups[colour], key=lambda b: b.pos[2])     # lowest cube
move   = max(groups[colour], key=lambda b: b.pos[2])     # highest cube

print(f"Stacking {move.name} on {base.name} (colour={colour})")

# ---------------------------------------------------------------- constants
TABLE_Z   = env.table_offset[2] + env.table_full_size[2] / 2
CUBE_H    = env.block_size[2]
HOVER_Z   = TABLE_Z + 0.15
GRASP_Z   = move.pos[2] + 0.015
STACK_Z   = base.pos[2] + CUBE_H     # one cube height above base

DOWN_QUAT = np.array([0, 1, 0, 0])   # tool pointing straight down (w,x,y,z)

# ---------------------------------------------------------------- sequence
goto(env, env._eef_xpos,                     DOWN_QUAT, gripper=-1)      # open
goto(env, [*move.pos[:2], HOVER_Z],          DOWN_QUAT, gripper=-1)      # hover
goto(env, [*move.pos[:2], GRASP_Z],          DOWN_QUAT, gripper=-1)      # descend
goto(env, env._eef_xpos,                     DOWN_QUAT, gripper=+1)      # close
goto(env, [*move.pos[:2], HOVER_Z],          DOWN_QUAT, gripper=+1)      # lift
goto(env, [*base.pos[:2], HOVER_Z],          DOWN_QUAT, gripper=+1)      # translate
goto(env, [*base.pos[:2], STACK_Z],          DOWN_QUAT, gripper=+1)      # lower
goto(env, env._eef_xpos,                     DOWN_QUAT, gripper=-1)      # release
goto(env, [*base.pos[:2], HOVER_Z],          DOWN_QUAT, gripper=-1)      # retreat

print("Done – one cube stacked.")
time.sleep(1.0)
env.close()