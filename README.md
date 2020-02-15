# virtualgimbal_ros
Inertial measurement Unit (IMU)で計測した角速度に基づいて、ビデオ映像のブレを補正して安定化します。OpenCLによるGPU処理によりリアルタイム動作が可能です。  
  
# 1. Overview  
このパッケージはvirtualgimbal_ros_nodeとsynchronizer_nodeからなります。virtualgimbal_ros_nodeはビデオ映像を安定化するノードです。  
<<ここにyoutube動画を追加>>  
# 2. Mini Tutorial
次のコマンドでvirtualgimbal_ros_nodeを起動します。
```
$ roslaunch virtualgimbal_ros stabilize_realsense_rgb.launch
```
サンプルとしてRealSense D435iのrosbagを用意しました。このrosbagを再生すると安定化される様子がわかります。  
```
$ rosbag play 2019-09-07-14-17-26.bag --clock
```
rqtで可視化しましょう。  
```  
$ rqt  
```  
<<■ここにrqtのイメージ画像>>  
# 3. Nodes
## 3.1
<<■ここにノード図を追加>>  
カメラで撮影した動画と、IMUで計測した角速度から、安定化した動画を生成します。

### 3.1.1 Subscribed Topics
image_raw (sensor_msgs/Image)  
  Rectified image stream from the camera driver.

camera_info (sensor_msgs/Camerainfo)
  Camera metadata.

img_data (sensor_msgs/Imu)
  Angular velocity.
 
# virtualgimbal_ros
Video stabilization using IMU information. This package stabilizes a video taken with a camera on mobile robots.
