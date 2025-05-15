import os
import numpy as np
from scipy.spatial.transform import Rotation as R

def quat_to_rt_matrix(w, x, y, z, px, py, pz):
    # Create a rotation matrix from the quaternion
    rotation = R.from_quat([x, y, z, w])  # Note: scipy uses [x, y, z, w] format
    rot_matrix = rotation.as_matrix()  # 3x3 rotation matrix

    # Construct the RT matrix
    rt_matrix = np.eye(4)
    rt_matrix[:3, :3] = rot_matrix
    rt_matrix[:3, 3] = [px, py, pz]

    return rt_matrix

def process_pose_quat_file(file_path):
    output_path = os.path.join(os.path.dirname(file_path), "pose_rt.txt")

    with open(file_path, 'r') as infile, open(output_path, 'w') as outfile:
        for line in infile:
            # Parse the line to extract quaternion and position
            values = list(map(float, line.strip().split()))
            if len(values) != 7:
                print(f"Skipping invalid line: {line.strip()}")
                continue

            w, x, y, z, px, py, pz = values
            rt_matrix = quat_to_rt_matrix(w, x, y, z, px, py, pz)

            # Write the RT matrix as a single line of 16 numbers to the output file
            rt_flat = rt_matrix.flatten()
            outfile.write(" ".join(map(str, rt_flat)) + "\n")

    print(f"RT matrices have been written to {output_path}")

if __name__ == "__main__":
    input_file = "/home/bobh/diablo/dataset_tum_format/pose_quat.txt"
    if os.path.exists(input_file):
        process_pose_quat_file(input_file)
    else:
        print(f"File {input_file} does not exist.")