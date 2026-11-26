"""両腕と首の標準関節名。値・校正はSDK設定から読む。"""
SIDES = ("right", "left")
ARMS = {"right": [f"R_joint{i}" for i in range(1, 8)],
        "left": [f"L_joint{i}" for i in range(1, 8)]}
GRIPPERS = {"right": "R_gripper_joint", "left": "L_gripper_joint"}
NECK = ["neck_pan_joint", "neck_tilt_joint"]
JOINTS = [name for side in SIDES for name in ARMS[side] + [GRIPPERS[side]]] + NECK
