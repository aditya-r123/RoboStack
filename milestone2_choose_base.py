"""
Milestone 2
===========

Differentiate between blocks of the *same* colour and choose which one will act
as the base of that colour's stack.

Steps
-----
1.  Reset the environment; grab a top-down RGB image ("birdview" camera).
2.  Detect coloured blocks in the image via HSV threshold (same as milestone 1).
3.  Query Robosuite for each block's 3-D world position.
4.  Group by colour and pick the block with the **lowest z-height** as the base
    (it's resting on the table already).
5.  Show the result: green boxes mark the chosen bases, red boxes mark the
    other blocks.
"""


import cv2
import numpy as np
import collections

import robosuite
import envs                                   # registers MultiColorBlockEnv

# ------------------------------ make env ----------------------------------
env = robosuite.make(
    "MultiColorBlockEnv",
    robots="Panda",
    has_renderer=False,
    has_offscreen_renderer=True,
    use_camera_obs=True,
    camera_names="frontview",      # or "agentview"
    render_camera="frontview",     # the one shown if you call env.render()
    camera_heights=480,
    camera_widths=640,
    camera_segmentations="element",   # <-- segmentation buffer we will use
)

obs = env.reset()
rgb = cv2.rotate(obs["frontview_image"], cv2.ROTATE_180)
seg_raw = cv2.rotate(obs["frontview_segmentation_element"], cv2.ROTATE_180)

# --------------------------------------------------- segmentation to int32
if seg_raw.ndim == 3:
    if seg_raw.shape[2] == 3:          # RGB-encoded 24-bit id
        seg_int = (seg_raw[:, :, 0].astype(np.int32) +
                   (seg_raw[:, :, 1].astype(np.int32) << 8) +
                   (seg_raw[:, :, 2].astype(np.int32) << 16))
    else:                              # shape (H, W, 1)  → squeeze
        seg_int = seg_raw[:, :, 0].astype(np.int32)
else:                                  # already (H, W)
    seg_int = seg_raw.astype(np.int32)
# seg_int is now 2-D int32 -----------------------------------------------

# ------------------------------ gather block info ------------------------
Block = collections.namedtuple(
    "Block", ["name", "rgba", "world_pos", "bbox_pix"]
)

blocks = []
for blk in env.blocks:
    rgba = tuple(blk.rgba)
    body_id = env.sim.model.body_name2id(blk.root_body)
    pos     = env.sim.data.body_xpos[body_id]

    # ------- NEW: locate geom(s) that belong to this body -------------
    geom_ids = np.where(env.sim.model.geom_bodyid == body_id)[0]
    if geom_ids.size == 0:
        continue                          # should not happen
    geom_id = int(geom_ids[0])            # take the first (visual) geom
    # element-segmentation stores id as geom_id + 1
    ys, xs = np.where(seg_int == geom_id + 1)
    bbox = None
    if xs.size:
        x1, x2, y1, y2 = xs.min(), xs.max(), ys.min(), ys.max()
        bbox = (x1, y1, x2 - x1, y2 - y1)

    blocks.append(Block(blk.name, rgba, pos, bbox))

# ------------------------------ select base per colour -------------------
groups = collections.defaultdict(list)
for b in blocks:
    groups[b.rgba].append(b)

bases = {}
for rgba, blist in groups.items():
    base = min(blist, key=lambda b: b.world_pos[2])   # lowest z
    bases[rgba] = base

print("Base blocks chosen (one per colour):")
for rgba, blk in bases.items():
    print(f"  {blk.name:<10}  rgba={rgba}  z={blk.world_pos[2]:.3f}")

# ------------------------------ visualise --------------------------------
vis_img = rgb.copy()
for blk in blocks:
    if blk.bbox_pix is None:
        continue
    x, y, w, h = blk.bbox_pix
    is_base = (blk in bases.values())
    colour = (0, 255, 0) if is_base else (0, 0, 255)   # green / red
    cv2.rectangle(vis_img, (x, y), (x + w, y + h), colour, 2)
    label = "BASE" if is_base else blk.name
    cv2.putText(vis_img, label, (x, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1)

cv2.imshow("Milestone-2  (green = chosen base)", cv2.cvtColor(vis_img, cv2.COLOR_RGB2BGR))
cv2.waitKey(0)
cv2.destroyAllWindows()

# rotate 90° counter-clockwise around the table-up (z) axis
env.robots[0].robot_model.set_base_ori([0, 0, np.pi/2])
env.sim.forward()            # commit pose change
env.render()

env.close()

rgb = cv2.rotate(rgb, cv2.ROTATE_180)       # turn it right-side-up
seg = cv2.rotate(seg, cv2.ROTATE_180)