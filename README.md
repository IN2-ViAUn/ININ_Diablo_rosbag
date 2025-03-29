# ROSBag 数据处理工具

## 项目简介

本项目旨在处理 ROSBag 数据，提供数据提取和同步保存功能。主要包括以下功能模块：

1. **提取数据为 TUM 格式**：通过 `bag_extractor` 提取轨迹和图像数据，保存为 TUM 格式文件。
2. **同步保存数据**：通过 `my_rosbag` 实现姿态、深度图像和 RGB 图像的同步保存。

---

## 文件结构与功能说明

### 1. [`src/rosbag_image_save/src/bag_extractor.cpp`](src/rosbag_image_save/src/bag_extractor.cpp)
- **功能**：
  - 从 ROSBag 数据中提取轨迹信息并保存为 TUM 格式的轨迹文件。
  - 提取 RGB 图像和深度图像，分别保存到指定目录。
  - 支持深度图像的单位转换（米到毫米）。
- **主要功能点**：
  - 订阅 `/state_estimation` 轨迹话题，保存轨迹到 `trajectory.txt`。
  - 订阅 `/rgb/image_raw` 和 `/depth_to_rgb/image_raw` 图像话题，保存图像到 `rgb` 和 `depth` 文件夹。
- **输出**：
  - `trajectory.txt`：TUM 格式轨迹文件。
  - `rgb` 和 `depth` 文件夹：保存的 RGB 和深度图像。

---

### 2. [`src/rosbag_image_save/src/image_saver.cpp`](src/rosbag_image_save/src/image_saver.cpp)
- **功能**：
  - 同步保存 RGB 图像和深度图像。
  - 将图像的时间戳和文件名保存为 JSON 格式文件。
- **主要功能点**：
  - 使用 `message_filters` 实现 `/rgb/image_raw` 和 `/depth_to_rgb/image_raw` 图像话题的时间同步。
  - 保存图像到 `rgb_images` 和 `depth_images` 文件夹。
  - 将时间戳和文件名记录到 [`timestamps.json`](timestamps.json) 文件。
- **输出**：
  - `rgb_images` 和 `depth_images` 文件夹：保存的 RGB 和深度图像。
  - [`timestamps.json`](timestamps.json)：记录图像时间戳和文件名的 JSON 文件。

---

### 3. [`src/rosbag_image_save/src/my_rosbag.cpp`](src/rosbag_image_save/src/my_rosbag.cpp)
- **功能**：
  - 同步保存姿态、深度图像和 RGB 图像。
  - 保存姿态的四元数和 RT 矩阵。
  - 将图像的时间戳和文件名保存为 JSON 格式文件。
- **主要功能点**：
  - 使用 `message_filters` 实现 `/state_estimation`、`/rgb/image_raw` 和 `/depth_to_rgb/image_raw` 的时间同步。
  - 保存姿态的四元数到 `pose_quat.txt`，RT 矩阵到 `pose_rt.txt`。
  - 保存图像到 `depth_images` 和 `rgb_images` 文件夹。
  - 将时间戳和文件名记录到 [`timestamps.json`](timestamps.json) 文件。
- **输出**：
  - `pose_quat.txt`：保存的姿态四元数。
  - `pose_rt.txt`：保存的姿态 RT 矩阵。
  - `depth_images` 和 `rgb_images` 文件夹：保存的 RGB 和深度图像。
  - [`timestamps.json`](timestamps.json)：记录图像时间戳和文件名的 JSON 文件。

---

## 更新记录

### 初次更新
- **时间**：2025 年 3 月 29 日
- **内容**：
  - 实现 `bag_extractor` 功能，支持轨迹提取和图像保存。
  - 实现 `image_saver` 功能，支持图像同步保存和时间戳记录。
  - 实现 `my_rosbag` 功能，支持姿态、图像同步保存及时间戳记录。

---

## 使用说明

1. **编译项目**：
   - 在工作空间根目录下运行以下命令：
     ```bash
     colcon build
     ```
2. **运行节点**：
   - 提取数据为 TUM 格式：
     ```bash
     ros2 run rosbag_image_save bag_extractor
     ```
   - 同步保存图像：
     ```bash
     ros2 run rosbag_image_save image_saver
     ```
   - 同步保存姿态和图像：
     ```bash
     ros2 run rosbag_image_save my_rosbag
     ```
3. **输出文件**：
   - 数据将保存到指定的输出目录（如 [rosbag_save_3_29](http://_vscodecontentref_/0)。

---

## 注意事项

- 确保 ROS2 环境已正确配置。
- 确保依赖库（如 OpenCV、cv_bridge、nlohmann/json）已安装。
- 修改保存路径时，请同步更新代码中的宏定义路径。

---

## 后续计划

- 增加对其他数据格式的支持。
- 优化同步策略，提高数据处理效率。
- 增加单元测试，确保代码稳定性。