# 激光雷达-相机标系外参计算

该方法适用于大场景下的联合标定，当利用autoware等工具捕获不了标定板的时候，我们选择通过鼠标监听事件手动选取3D，2D点

最后利用OpenCV的PnP求解激光雷达到相机坐标系的外参矩阵