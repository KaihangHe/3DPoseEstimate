# MultipleCameraSFM

##### by Nicapoet
---
### depends：
- opencv 4.0.1 
     * contrib
     * viz
---
##### Calibrator
_相机标定_
1. 相机标定类已完成，每张标定图像的红色角点行是x轴，自红色向蓝色变化是y轴,每行连接线角度开口是原点方向
2. Calibrator构造函数：
注意：

    - cv::Size square_size是棋盘小方格尺寸，单位mm
    > https://blog.csdn.net/xueluowutong/article/details/80950915

    ```
        Calibrator(const char *images_folder,cv::Size board_size,cv::Size square_size);
    ```
    - Calibrator构造函数会输出重投影误差
##### CloudPtsGenerator
_点云生成_
1. update_Pose方法返回三维点云齐次坐标，如需笛卡尔三维坐标请使用tran_Mat_2_Point3d方法转换 
##### FeaturePtsCatcher
_特征点对捕获_
1. readImagePoints方法从yml中序列化读取Point2f数据
##### GireCamera
_工业相机驱动_
##### VizViewer
_三维可视化模块_

 