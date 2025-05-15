# Diablo Robot Dataset Generator (TUM Format)

## Motion Capture workflow

To use this workflow, please ensure that your rosbag contains motion capture pose topic

And then, run `tum_dataset_generator/src/rosbag_image_save/my_rosbag.cpp` as ROS2 Node

remember to change the following topic name and saving path:

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

After exporting the raw data by playing the rosbag, you should export the `pose_rt.txt` by running:

```bash
python quat_to_rt.py
```

Still, remember to modify the path in this python script.

Then, to export the `groundtruth.txt` in tum format, run:

```bash
python generate_real_groundtruth.py
```

Remember to modify the following code:

```python
POSE_RT_PATH = "/home/bobh/diablo/dataset_tum_format/pose_rt.txt"
OUTPUT_GT_PATH = "/home/bobh/diablo/dataset_tum_format/groundtruth.txt"
RGB_PATH = "/home/bobh/diablo/dataset_tum_format/rgb.txt"
NUM_FRAMES = 3496
```

The most important thing is to put the correct `hand_eye_calibration_result.npz` in this folder, you could gain this file by referring the project: [mocap_cam_handeye_calibration
](https://github.com/IN2-ViAUn/mocap_cam_handeye_calibration)

Then you should get the full TUM format dataset with `groundtruth.txt`, `rgb.txt` and `depth.txt`

## Lidar SLAM workflow

To use this workflow, please ensure that your rosbag contains pose observations provided by the SLAM algorithm.

And then, run `tum_dataset_generator/src/rosbag_image_save/my_rosbag_SLAM.cpp` as ROS2 Node

remember to change the following topic name and saving path:

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

`/init_gravity_pose` this gravity pose is for gravity adjustment. If you are unsure about this, simply set it to the Unit transformation.

After exporting the raw data by playing the rosbag, you should export the `pose_rt.txt` by running:

```bash
python quat_to_rt.py
```

And then export the `cam_pose_rt_gravity_fixed.txt` by running:

```bash
python fix_gravity_pose.py
```

And then export the `groundtruth.txt` by running:

```bash
python generate_slam_real_groundtruth.py
```

Remember to modify the following parameters:

```python
POSE_RT_PATH = "/home/bobh/diablo/dataset_tum_format/cam_pose_rt_gravity_fixed.txt"
OUTPUT_GT_PATH = "/home/bobh/diablo/dataset_tum_format/groundtruth.txt"
RGB_PATH = "/home/bobh/diablo/dataset_tum_format/rgb.txt"
NUM_FRAMES = 844
```

Then you should get the full TUM format dataset with `groundtruth.txt`, `rgb.txt` and `depth.txt`
