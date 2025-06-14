import numpy as np
import cv2
import robosuite
import envs


env = robosuite.make(
    "MultiColorBlockEnv",
    robots="Panda",
    has_renderer=False,
    has_offscreen_renderer=True,
    use_camera_obs=True,
    camera_names="agentview",
    camera_heights=256,
    camera_widths=256,
    camera_segmentations=None,
)

obs = env.reset()
rgb = obs["agentview_image"]

hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)

RANGES = {
    "red":     [((0,  70, 50), (10, 255, 255)),
                ((170,70, 50), (180,255,255))],
    "green":   [((45, 40, 40), (85, 255, 255))],
    "blue":    [((100,150, 0), (140,255,255))],
    "yellow":  [((20,100,100), (35, 255, 255))],
    "magenta": [((140,50,50),  (170,255,255))],
}

detections = []
for colour, ranges in RANGES.items():
    mask_total = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for (lo, hi) in ranges:
        mask_total |= cv2.inRange(hsv, lo, hi)
    mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN,
                                  np.ones((3, 3), np.uint8))

    contours, _ = cv2.findContours(mask_total,
                                   cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        if cv2.contourArea(c) < 100:
            continue
        x, y, w, h = cv2.boundingRect(c)
        detections.append((colour, (x, y, w, h)))

print("Detections:")
for colour, bb in detections:
    print(f"  {colour:<7}  bbox={bb}")

for colour, (x, y, w, h) in detections:
    cv2.rectangle(rgb, (x, y), (x + w, y + h),
                  (0, 255, 0), 2)
    cv2.putText(rgb, colour, (x, y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
cv2.imshow("detections", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
cv2.waitKey(0)
cv2.destroyAllWindows()

env.close()