# Diablo机器人数据集生成器（TUM格式）

[English Version (英文版)](README_en.md)

## 动作捕捉工作流

使用此工作流前，请确保您的rosbag包含动作捕捉姿态话题

然后运行`tum_dataset_generator/src/rosbag_image_save/my_rosbag.cpp`作为ROS2节点

请修改以下话题名称和保存路径：

```cpp
#define POSE_TOPIC1 "/vrpn_mocap/Azure_Cam0/pose"
#define RGB_TOPIC "/rgb/image_raw"
#define DEPTH_TOPIC "/depth_to_rgb/image_raw"
#define SAVE_PATH_ROOT "/home/bobh/diablo/dataset_tum_format"
#define POSE_SAVE_QUAT "/home/bobh/diablo/dataset_tum_format/pose_quat.txt"
#define POSE_SAVE_RT "/home/bobh/diablo/dataset_tum_format/pose_rt.txt"

# define SAVE_DEPTH_IMAGE_PATH "/home/bobh/diablo/dataset_tum_format/depth"
# define SAVE_RGB_IMAGE_PATH "/home/bobh/diablo/dataset_tum_format/rgb"
```

播放rosbag导出原始数据后，您需要运行以下命令生成`pose_rt.txt`：

```bash
python quat_to_rt.py
```

同样需要修改此Python脚本中的路径。

然后运行以下命令生成TUM格式的`groundtruth.txt`：

```bash
python generate_real_groundtruth.py
```

请修改以下代码：

```python
POSE_RT_PATH = "/home/bobh/diablo/dataset_tum_format/pose_rt.txt"
OUTPUT_GT_PATH = "/home/bobh/diablo/dataset_tum_format/groundtruth.txt"
RGB_PATH = "/home/bobh/diablo/dataset_tum_format/rgb.txt"
NUM_FRAMES = 3496
```

最重要的是将正确的`hand_eye_calibration_result.npz`文件放在此文件夹中，您可参考此项目获取该文件： [mocap_cam_handeye_calibration
](https://github.com/IN2-ViAUn/mocap_cam_handeye_calibration)

完成后您将获得完整的TUM格式数据集，包含`groundtruth.txt`、`rgb.txt`和`depth.txt`

## 激光雷达SLAM工作流

使用此工作流前，请确保您的rosbag包含SLAM算法提供的位姿观测

然后运行`tum_dataset_generator/src/rosbag_image_save/my_rosbag_SLAM.cpp`作为ROS2节点

请修改以下话题名称和保存路径：

```cpp
#define POSE_TOPIC1 "/state_estimation"
#define RGB_TOPIC "/rgb/image_raw"
#define DEPTH_TOPIC "/depth_to_rgb/image_raw"
#define SAVE_PATH_ROOT "/home/bobh/diablo/dataset_tum_format"
#define POSE_SAVE_QUAT "/home/bobh/diablo/dataset_tum_format/pose_quat.txt"
#define POSE_SAVE_RT "/home/bobh/diablo/dataset_tum_format/pose_rt.txt"


#define SAVE_DEPTH_IMAGE_PATH "/home/bobh/diablo/dataset_tum_format/depth"
#define SAVE_RGB_IMAGE_PATH "/home/bobh/diablo/dataset_tum_format/rgb"

#define SAVE_GRAVITY_POSE_PATH "/home/bobh/diablo/dataset_tum_format/gravity_pose.txt"
```

`/init_gravity_pose`用于重力调整。如果不确定，只需将其设置为单位变换。

播放rosbag导出原始数据后，运行以下命令生成`pose_rt.txt`：

```bash
python quat_to_rt.py
```

然后运行以下命令生成`cam_pose_rt_gravity_fixed.txt`：

```bash
python fix_gravity_pose.py
```

最后运行以下命令生成`groundtruth.txt`：

```bash
python generate_slam_real_groundtruth.py
```

请修改以下参数：

```python
POSE_RT_PATH = "/home/bobh/diablo/dataset_tum_format/cam_pose_rt_gravity_fixed.txt"
OUTPUT_GT_PATH = "/home/bobh/diablo/dataset_tum_format/groundtruth.txt"
RGB_PATH = "/home/bobh/diablo/dataset_tum_format/rgb.txt"
NUM_FRAMES = 844
```

完成后您将获得完整的TUM格式数据集，包含`groundtruth.txt`、`rgb.txt`和`depth.txt`