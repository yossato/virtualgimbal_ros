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
![nodes](https://github.com/yossato/images/blob/master/nodes.png?raw=true)  
カメラで撮影した動画と、IMUで計測した角速度から、安定化した動画を生成します。

### 3.1.1 Subscribed Topics
image_raw (sensor_msgs/Image)  
  Rectified image stream from the camera driver.

camera_info (sensor_msgs/Camerainfo)
  Camera metadata.

img_data (sensor_msgs/Imu)
  Angular velocity.  

## 3.1.2 Published Topics
stabilized/image_rect (sensor_msgs/Image)  
  Stabilized image stream.  
  
stabilized/camera_info (sensor_msgs/CameraInfo)
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

# 4. Launch files
## 4.1 stabilize.launch
イメージストリームを安定化します。  

### 4.2 stabilize_realsense_rgb.launch
Intel RealSense D435iのRGBイメージストリームについてIMUの角速度を用いて安定化します。paramとしてD435iのトピック名を指定してstabilize.launchを起動します。

### stabilize_realsense_ir.launch
Intel RealSense D435iのirカメラの左側のイメージストリームについてIMUの角速度を用いて安定化します。paramとしてD435iのトピック名を指定してstabilize.launchを起動します。

### synchronizer.launch
画像ストリームとIMUの時刻同期をします。イメージストリームについてIMUの角速度と同期のタイミングを計算します。カメラ画像のタイムスタンプは、時刻の定義方法により値が変わるため、IMUを利用して安定化するときに問題になります。virtualgimbal_rosではカメラの露光中心をタイムスタンプの基準として利用しています。このlaunchにより、correlation_timeに指定された秒数だけデータが集まると、sum of absolute differences (SAD)により相関を計算することで、画像とIMUについて時刻を変えながら最も相関が高いタイミングを計算します。得られた値はstabilize.launchのparamのoffset_timeにセットしてください。

### synchronize_realsense_rgb.launch
Intel RealSense D435iのRGBイメージストリームとIMUの同期を取ります。

### synchronize_realsense_ir.launch
RGBと同様にIRイメージストリームについてもIMUとの同期を取ります。


