# 激光雷达-相机坐标系外参计算

## 适用情况

该方法适用于大场景下的联合标定，利用Autoware等工具捕获不了标定板的情况



## 主要过程

- 通过鼠标监听事件**手动选取**3D，2D点
- 利用OpenCV的**PnP**求解激光雷达到相机坐标系的**外参矩阵**



## 使用方式

需要修改四个参数

- `save_path`：激光雷达-相机外参存储路径
- `thermal_calib`：相机内参路径
- `img_path`：图像路径
- `filename`：点云路径(txt格式)

注意：内参和外参矩阵都用xml格式进行了存储

---

详细介绍可以参考我的博客

https://blog.csdn.net/HelloJinYe/article/details/117637392