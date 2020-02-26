# virtualgimbal_ros
Inertial measurement Unit (IMU)で計測した角速度に基づいて、ビデオ映像のブレを補正して安定化します。  
カメラ映像を通じたロボット操縦時の視認性向上、画像認識・物体追跡性能向上および映像撮影品質の向上を目的としています。  
1920 x 1080 pixel, 30 fpsのRGB動画に対して、OpenCLによるGPU処理によりノートPCのCPU(Intel Core i7-8550U)内蔵GPUでリアルタイムで安定化が動作することを確認しています。  
  
# 1. Overview  
このパッケージは、動画をジャイロセンサで計測した角速度で安定化するvirtualgimbal_ros_nodeと、動画とジャイロセンサ間のタイムスタンプのオフセットを精密測定するsynchronizer_nodeからなります。  
[![](https://img.youtube.com/vi/ft6v7h5kN6g&feature=youtu.be/0.jpg)](https://www.youtube.com/watch?v=ft6v7h5kN6g&feature=youtu.be)  

# 1.1 Install dependencies  
本パッケージは Ubuntu 16.04 と ROS Kinetic で動作確認をしました。  
動作にはOpenCLのセットアップが必要です。  

```
# apt install ocl-icd-libopencl1 opencl-headers clinfo ocl-icd-opencl-dev  
# add-apt-repository ppa:intel-opencl/intel-opencl
# apt update
# apt install intel-opencl-icd
```


# 2. Mini Tutorial
[RealSense D435iで撮影した動画のrosbag](https://drive.google.com/uc?id=1uaRjiDGhFVExLlXTnzp7vVMDJGi7vvzC)をダウンロードして安定化動作を試すことができます。以下のコマンドを別々のターミナルで実行してください。    
```
$ roslaunch virtualgimbal_ros stabilize_realsense_rgb.launch  
$ rosbag play 2019-09-07-14-17-26.bag --clock  
```
rqtで可視化しましょう。  
```  
$ rqt  
```  
左が安定化前の動画で右が安定化後の動画です。安定化後は画像が拡大されて周囲が切り取られますが画像内の動きが少なく動きが安定化しています。  
![rqt_image](https://github.com/yossato/images/blob/master/soleil-hill-park.png?raw=true) 
  
# 3. Nodes
## 3.1 virtualgimbal_ros_node  
![nodes](https://github.com/yossato/images/blob/master/nodes.png?raw=true)  
カメラで撮影した動画と、IMUで計測した角速度から、安定化した動画を生成します。

### 3.1.1 Subscribed Topics
#### image_raw (sensor_msgs/Image)  
Rectified image stream from the camera driver.
  
#### camera_info (sensor_msgs/Camerainfo)  
Camera metadata.
  
#### imu_data (sensor_msgs/Imu)  
Angular velocity.  
  
## 3.1.2 Published Topics
#### stabilized/image_rect (sensor_msgs/Image)  
Stabilized image stream.  
  
#### stabilized/camera_info (sensor_msgs/CameraInfo)  
Stabilized camera metadata.  
  
## 3.1.3 Parameter  
|Parameter|Type|default|description|
|:---|:---|:---|:---|
|image|string|image|入力画像トピック|
|imu_data|string|imu_data|入力角速度トピック|
|zoom_factor|float|1.3|画像のズーム倍率。1以上の値を設定する。値を大きくすると画像が拡大され手ブレ補正能力が向上するが、画像の四隅の切り取られる量が増える。|
|enable_trimming|bool|true|trueに設定すると画像の四隅を切り取った分だけ出力画像サイズを小さくする。falseに設定すると入力画像と出力画像のサイズを等しくする。出力画像は引き伸ばされる。trueにしたほうがtopicのデータ量は減少するが画像サイズが一般的ではないサイズになる。|
|offset_time|double|0|ジャイロセンサと画像ストリームのタイムスタンプのオフセット時間(秒)。synchronizer_nodeにより計測できる。|
|verbose|bool|false|デバック機能を提供する。trueで各種ログが有効化され、安定化の様子がグラフで表示される。falseでログ機能を停止する。|
|allow_blue_space|bool|false|trueで画像に青い部分が生じることを許すと、ブレ補正能力が大幅に向上しますがカメラのブレが大きくなり限界を迎えると画面の端に青い部分が生じます。falseにするとブレ補正能力が低下しますが青い部分ができないようにします。|
|lsm_period|double|1.5|最小二乗法(least squares method)を計算する時間の長さ(秒)。値を大きくすると安定化能力が向上するが、急なカメラの動きに追従できなくなる。値を小さくすると安定化能力が低下するがカメラの急な動きに追従できるようになる。|
|lsm_order|double|1|最小二乗法でフィッティングする曲線の次数。1だと1次式の直線でフィッティングする。2だと2次式の放物線でフィッティングする。次数を上げると追従性が向上するが安定化能力が低下する。|
  
## 3.1 synchronizer_node  
動画とIMU間のタイムスタンプのオフセットを精密測定するノードです。動画のタイムスタンプはカメラのシャッターの露光タイミングの定義方法により変わります。そのため、IMUを利用して安定化するときに、定義の違いによるタイミングのわずかな差が安定化品質の問題になります。動画の安定化にはミリ秒以下の精度の同期が必要になります。  
virtualgimbal_rosではカメラのシャッターの露光中心をタイムスタンプの基準として利用しています。このnodeはSum of Absolute Differences (SAD)により動画とIMUの角速度の相関を計算し最も良く相関があるタイミングを計算します。もう少し詳しく説明すると動画からオプティカルフローを計算し、オプティカルフローから角速度を推定し、推定された角速度と、IMUにより計測された角速度の相関を、時間を少しずつ変化させながら計算します。得られたオフセットはstabilize.launchのparamのoffset_timeにセットして使います。  
  
### 3.1.1 Subscribed Topics
#### image_raw (sensor_msgs/Image)  
Rectified image stream from the camera driver.
  
#### camera_info (sensor_msgs/Camerainfo)  
Camera metadata.
  
#### imu_data (sensor_msgs/Imu)  
Angular velocity.  
  
### 3.1.2 
Parameter  
|Parameter|Type|default|description|
|:---|:---|:---|:---|
|image|string|image|入力画像トピック|
|imu_data|string|imu_data|入力角速度トピック|
|maximum_offset_time|float|0.5|SADを計算するオフセットの最大値(秒)。動画とIMUのタイムスタンプ差の最大値を指定してください。値を大きくすると動画とIMUのタイムスタンプの誤差が大きくてもオフセットの推定が可能ですが、計算に時間がかかるようになります。|
|correlation_time|float|15.0|SADを計算する時間の長さ(秒)。長くするとオフセットの推定精度が向上するが、推定に必要な動画の長さが長くなり、推定に時間がかかるようになる。|

# 4. Launch files
## 4.1 stabilize.launch
イメージストリームを安定化します。  

### 4.2 stabilize_realsense_rgb.launch
Intel RealSense D435iのRGBイメージストリームについてIMUの角速度を用いて安定化します。paramとしてD435iのトピック名を指定してstabilize.launchを起動します。

### stabilize_realsense_ir.launch
Intel RealSense D435iのirカメラの左側のイメージストリームについてIMUの角速度を用いて安定化します。paramとしてD435iのトピック名を指定してstabilize.launchを起動します。

### synchronizer.launch
画像ストリームとIMUの時刻同期をします。イメージストリームについてIMUの角速度と同期のタイミングを計算します。

### synchronize_realsense_rgb.launch
Intel RealSense D435iのRGBイメージストリームとIMUの同期を取ります。

### synchronize_realsense_ir.launch
RGBと同様にIRイメージストリームについてもIMUとの同期を取ります。


