# 基于BoW的ORB特征地铁零件识别系统

## 一、环境安装配置

### 1.1系统环境安装配置

1）开发环境：Core i3以上CPU，内存2GB 以上。

2）运行环境： Raspberry OS安装Pangolin、Eigen、Sophus、PCL、OpenCV、g2o、ceres、glog、gflag、libfreenect2、glew、libQGLViewer、libusb、suitesparse-metis、DBoW3等所需库并配置gcc编译环境

## 二、系统说明

### 2.1软件概述

本软件基于由ORB特征和BoW词袋法技术，实现了由二维照片检测并检索地铁零件的系统。

本软件识别框架如下图所示：

![image-20230528174902335](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-174902-YcxQpR.png)

图 物体识别技术路线图

 

### 2.2 完成主要功能

1.离线训练

2.实时检测

3.蜂鸣器报警，在检测到物体时会发出“哔—”的声音，在训练时拍摄照片成功也会发出“哔—”的声音。

4.当识别到物体时，在屏幕上展示识别到物体的当前帧，并且在图像上显示物体的名称。

5.具有温湿度传感器，并且在LED屏幕上显示当前环境的温湿度。

6.有是否检测过的标识，用户可在检测后查看isDetec.txt来了解物体是否都检查过。

![image-20230528174924504](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-174924-uouJd3.png)

图 训练过程

![image-20230528174938328](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-174938-zj68Pr.png)

（a）未识别到物体，红色指示灯亮起    

![image-20230528174954661](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-174954-BNwxhh.png)

（b）识别到物体绿色指示灯亮起

## 三、操作说明

### 3.1 训练过程

运行程序出现如下图所示界面，输入1和回车进入图像训练过程。

![image-20230528175009247](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-175009-PEV9wo.png)

输入零件编号，按回车确定。

![image-20230528175017954](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-175018-LNzM9X.png)

此例中输入shuibei

弹出图像检测框，在图像检测框中，按空格键可以拍摄照片。

 ![image-20230528175032568](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-175032-siq8bg.png)

下图为拍摄的图片的路径。

![image-20230528175043641](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-175043-DAHwud.png)

按下“a”键，可结束当前零件的拍摄输入下一个零件的名称并按回车可继续进行拍摄：

![image-20230528175054282](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-175054-86wz9o.png)

当所有零件拍摄结束后，按下“q”键退出拍摄，程序会自动提取每张图片的ORB特征，并且将其训练和保存到词袋中。

![image-20230528175102704](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-175102-QXdF9Z.png)

如下图所示，即为训练结束后的词袋信息。

![image-20230528175111306](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-175111-zmblF6.png)

### 3.2 检测过程

运行程序，输入2和回车即可开始检测过程。

![image-20230528175120317](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-175120-Gkev7d.png)

程序会自动加载之前训练好的词库数据库

![image-20230528175128968](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-175129-piqKWd.png)

训练中若未发现匹配的零件，则会出现如下的提示：

![image-20230528175139077](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-175139-4OGWKV.png)

若出现了待检测的物体，则弹出检测框，并且在图像左上角标明待检测零件的名称。

![image-20230528175147655](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-175147-FCPiSP.png)\

![image-20230528175159038](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-175159-1tj9Mp.png)

![image-20230528175212463](https://cdn.jsdelivr.net/gh/inusturbo/images@main/uPic/20230528-175212-Ftd0G8.png)