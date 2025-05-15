import numpy as np
import open3d as o3d
import json
import cv2
from scipy.spatial.transform import Rotation as R

POSE_RT_PATH = "/home/bobh/diablo/dataset_tum_format/pose_rt.txt"
OUTPUT_GT_PATH = "/home/bobh/diablo/dataset_tum_format/groundtruth.txt"
RGB_PATH = "/home/bobh/diablo/dataset_tum_format/rgb.txt"
NUM_FRAMES = 3496

def load_hand_eye_calibration(filename="hand_eye_calibration_result.npz"):
    data = np.load(filename)
    T_G2C = data["hand_eye_transform"]  # shape (4,4)
    return T_G2C

def load_timestamps(rgb_txt_path):
    timestamps = []
    with open(rgb_txt_path, "r") as f:
        lines = f.readlines()
        for line in lines:
            if line.startswith("#") or not line.strip():
                continue  # Skip comments and empty lines
            timestamp = line.split()[0]  # Extract the timestamp
            timestamps.append(timestamp)
    return timestamps

# read camera pose from file pose_rt.txt, select i th line, return as 4x4 matrix
def get_robot_pose_for_frame(i):
    with open(POSE_RT_PATH, "r") as f:
        lines = f.readlines()
        if i < 0 or i >= len(lines):
            raise IndexError("Index out of range")
        line = lines[i].strip().split()
        pose = np.array([float(x) for x in line]).reshape(4, 4)
    return pose

def main():
    T_G2C = load_hand_eye_calibration("hand_eye_calibration_result.npz")
    timestamps = load_timestamps(RGB_PATH)
    ground_truth_result = "# ground truth trajectory\n# file: xxx\n# timestamp tx ty tz qx qy qz qw\n"
    for i in range(NUM_FRAMES):
        T_B2G_i = get_robot_pose_for_frame(i)
        T_B2C_i = T_B2G_i @ T_G2C
        # output T_B2C_i to ground_truth_result as tx ty tz qx qy qz qw
        tx, ty, tz = T_B2C_i[:3, 3]
        rotation_matrix = T_B2C_i[:3, :3]
        quaternion = R.from_matrix(rotation_matrix).as_quat()  # Returns [qx, qy, qz, qw]
        qx, qy, qz, qw = quaternion
        timestamp = timestamps[i]  # Get the corresponding timestamp
        ground_truth_result += f"{timestamp} {tx} {ty} {tz} {qx} {qy} {qz} {qw}\n"
    # write ground_truth_result to file
    with open(OUTPUT_GT_PATH, "w") as f:
        f.write(ground_truth_result)

if __name__ == "__main__":
    main()