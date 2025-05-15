import numpy as np

input_txt = "/home/bobh/diablo/dataset_tum_format/pose_rt.txt"
output_txt = "/home/bobh/diablo/dataset_tum_format/cam_pose_rt.txt"

matrix = np.array([
 [ 0.0109665,   0.31768448,  0.94813306, -0.0498735, ],
 [-0.99986395,  0.01516792,  0.00648263, -0.01692035,],
 [-0.01232178, -0.94807516,  0.3178076,   0.1390413, ],
 [ 0.,          0.,          0.,          1.,        ],
])
# 计算逆矩阵
inv_matrix = np.linalg.inv(matrix)

# 假设 trajectories 是从 traj.txt 文件中读取的所有 4x4 矩阵组成的列表
with open(input_txt, 'r') as file:
    lines = file.readlines()

trajectories = []
for line in lines:
    # 将每行转换成 4x4 的矩阵
    matrix_line = np.fromstring(line, sep=' ').reshape(4, 4)
    trajectories.append(matrix_line)

# 应用逆矩阵并保存新轨迹到 trajnew.txt
with open(output_txt, 'w') as new_file:
    for traj in trajectories:
        transformed_traj =  traj @ matrix  # 左乘逆矩阵
        
        # 将变换后的矩阵展平成一维数组
        flat_transformed_traj = transformed_traj.flatten()
        
        # 将展平的数组转换为字符串，并去除多余的空格和换行符
        line_to_write = np.array2string(flat_transformed_traj, separator=' ', max_line_width=np.inf)[1:-1].replace('\n', '')
        new_file.write(line_to_write + '\n')
    new_file.flush()