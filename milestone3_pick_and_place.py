"""
Milestone 3
-----------

Pick one non-base block, lift it vertically, translate at a safe hover
height, then stack it on top of its colour's base block.

• uses element-segmentation (perfect masks) from milestone 2
• OSC-Pose controller drives the gripper
• works with the default Panda + gripper
"""

import numpy as np, cv2, collections, time
import robosuite, envs
from robosuite.utils.transform_utils import quat2mat, mat2quat

# ------------------------------------------------------------------ helpers
def move_ee(env, target_pos, target_quat, gripper=0.0,
            speed=0.03, thresh=2e-3):
    robot    = env.robots[0]
    ctrl_dim = robot.controller.control_dim

    while True:
        cur_pos = env._eef_xpos
        dpos    = target_pos - cur_pos
        dist    = np.linalg.norm(dpos)

        if dist < thresh:                       # <- position only
            break

        step = dpos * speed                    # proportional step
        action = np.zeros(env.action_spec[0].shape)
        action[:3] = step                      # xyz
        # keep orientation constant ⇒ zeros in rot part
        if robot.gripper:
            action[-1] = gripper

        env.step(action)
        env.render()

    # final gripper command (open / close) even when already at target
    if robot.gripper:
        a = np.zeros(env.action_spec[0].shape)
        a[-1] = gripper
        env.step(a)
        env.render()

# ------------------------------------------------------------------ build env
env = robosuite.make(
    "MultiColorBlockEnv",
    robots="Panda",
    has_renderer=True,
    has_offscreen_renderer=True,
    use_camera_obs=True,
    camera_names="frontview",
    camera_heights=480,
    camera_widths=640,
    camera_segmentations="element",
    horizon=2000,          # plenty of room for slow scripted motion
    ignore_done=True,      # <– keep stepping past horizon
)
obs = env.reset()
env.render()          # opens the window

# ------------------------------------------------------------------ get block poses + choose bases
seg_raw = obs["frontview_segmentation_element"]      # could be H×W, H×W×1, H×W×3

if seg_raw.ndim == 3:
    if seg_raw.shape[2] == 3:            # RGB-encoded 24-bit id
        seg_int = (seg_raw[:, :, 0].astype(np.int32) +
                   (seg_raw[:, :, 1].astype(np.int32) << 8) +
                   (seg_raw[:, :, 2].astype(np.int32) << 16))
    else:                                # shape (H, W, 1)
        seg_int = seg_raw[:, :, 0].astype(np.int32)
else:                                    # already (H, W)
    seg_int = seg_raw.astype(np.int32)
# seg_int is now 2-D int32

Block = collections.namedtuple("Block", "name rgba pos geom_id")
blocks = []
for blk in env.blocks:
    body_id = env.sim.model.body_name2id(blk.root_body)
    geom_id = np.where(env.sim.model.geom_bodyid == body_id)[0][0]
    blocks.append(Block(blk.name, tuple(blk.rgba),
                        env.sim.data.body_xpos[body_id], geom_id))

# choose base = lowest z for each colour
groups = collections.defaultdict(list)
for b in blocks:
    groups[b.rgba].append(b)
bases = {rgba: min(lst, key=lambda b: b.pos[2]) for rgba, lst in groups.items()}

# pick one colour with >1 block (there's something to move)
colour_to_move = next(k for k,v in groups.items() if len(v) > 1)
base           = bases[colour_to_move]
moving         = next(b for b in groups[colour_to_move] if b != base)

print(f"Moving {moving.name}  -> stack on  {base.name}")

# ------------------------------------------------------------------ constants
TABLE_Z   = env.table_offset[2] + env.table_full_size[2]/2
HOVER_Z   = TABLE_Z + 0.15         # safe height above every tower
GRASP_Z   = moving.pos[2] + 0.015  # slightly above cube centre
STACK_Z   = base.pos[2] + 0.04     # height of one cube

DOWN_QUAT = np.array([1, 0, 0, 0]) # tool pointing down (w,x,y,z)

# ------------------------------------------------------------------ sequence
# 1. open gripper
move_ee(env, env._eef_xpos, env._eef_xquat, gripper=-1)

# 2. hover above the moving block
move_ee(env, np.array([moving.pos[0], moving.pos[1], HOVER_Z]),
        DOWN_QUAT, gripper=-1)

# 3. descend to grasp height
move_ee(env, np.array([moving.pos[0], moving.pos[1], GRASP_Z]),
        DOWN_QUAT, gripper=-1)

# 4. close gripper (grasp)
move_ee(env, env._eef_xpos, env._eef_xquat, gripper=1)
time.sleep(0.2)


# 5. lift straight up
move_ee(env, np.array([moving.pos[0], moving.pos[1], HOVER_Z]),
        DOWN_QUAT, gripper=1)

# 6. move above the base block
move_ee(env, np.array([base.pos[0], base.pos[1], HOVER_Z]),
        DOWN_QUAT, gripper=1)

# 7. lower to stack height
move_ee(env, np.array([base.pos[0], base.pos[1], STACK_Z]),
        DOWN_QUAT, gripper=1)

# 8. open gripper (release)
move_ee(env, env._eef_xpos, env._eef_xquat, gripper=-1)
time.sleep(0.2)

# 9. retreat up
move_ee(env, np.array([base.pos[0], base.pos[1], HOVER_Z]),
        DOWN_QUAT, gripper=-1)

print("Done – one block stacked.")
env.close() 