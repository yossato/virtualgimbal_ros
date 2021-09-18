# virtualgimbal_ros
VirtualGimbal ROSは、Inertial measurement Unit (IMU)で計測した角速度に基づいてブレを補正して、動画を安定化します。  
VirtualGimbal ROSは、カメラ映像を通じたロボット操縦時の視認性向上、画像認識・物体追跡性能向上および映像撮影品質の向上を目的として開発されました。  
VirtualGimbal ROSはOpenCLによるGPU処理で高速に動作します。例えば、Intel RealSense D435iの1920x1080 pixel, 30 fpsの動画をノートPC(Core i7-8550U)でリアルタイムに安定化できることを確認しています。  
  
# 1. Overview  
このパッケージは3種類のノードから成り立っています。  
  
1.動画をジャイロセンサで計測した角速度で安定化するvirtualgimbal_ros_node  
2.動画とジャイロセンサ間のタイムスタンプのオフセットを精密測定するsynchronizer_node  
3.ローリングシャッターのCMOSセンサの行ことに異なる読み出し遅延時間を精密測定するline_delay_estimator_node  
   
[![](https://github.com/yossato/images/blob/master/youtube.png?raw=true)](https://www.youtube.com/watch?v=ft6v7h5kN6g&feature=youtu.be) 
  
# 1.1 Install dependencies  
本パッケージは Ubuntu 20.04 と ROS noetic で動作確認済みです。    
動作には次のOpenCL関係のライブラリが必要です。  

```
# apt install ocl-icd-libopencl1 opencl-headers clinfo ocl-icd-opencl-dev  
```
加えて、GPUの種類に応じたドライバのインストールが必要です。  

### 1.1.1 NVIDIA GPU  
NVIDIAのGPUを使用する場合は[ディスプレイドライバ](https://www.nvidia.com/download/index.aspx)をインストールしてください。  

### 1.1.2 Intel HD Graphics  
IntelのCPU内蔵GPUを使う場合は次のコマンドによりOpenCL用のドライバである[Intel(R) Graphics Compute Runtime for OpenCL(TM)](https://github.com/intel/compute-runtime/blob/master/documentation/Neo_in_distributions.md)をインストールしてください。  
```
# add-apt-repository ppa:intel-opencl/intel-opencl
# apt update
# apt install intel-opencl-icd
```

### 1.1.3 AMD
T.B.D.  

# 2. Tutorials
## 2.1 Stabilizer Tutorial
サンプル動画で安定化の動作を体験してみます。まず[RealSense D435iで撮影した動画のrosbag](https://www.dropbox.com/s/43ucvmjfjhxeyzg/soleil.bag?dl=1)をダウンロードします。  
以下のコマンドを別々のターミナルで実行してください。    
```
$ roslaunch virtualgimbal_ros stabilize_realsense_rgb.launch  
$ rosbag play soleil.bag --clock  # rosbagの置かれたディレクトリで実行  
```
rqtのimage_viewの画面が2個起動します。
Input imageが安定化前の動画で、Stabilized Imageが安定化後の動画です。安定化後は画像が拡大されて周囲が切り取られますが画像内の動きが少なく動きが安定化しています。  
    
![Input Image ](https://github.com/yossato/images/blob/master/Screenshot%20from%202020-02-29%2023-10-14.png?raw=true)
![Stabilized Image ](https://github.com/yossato/images/blob/master/Screenshot%20from%202020-02-29%2023-10-10.png?raw=true)  
  
## 2.2 Line Delay Estimation Tutorial  
サンプル動画でローリングシャッターのCMOSカメラのLine delayの推定を体験してみます。Line delayの値は動画の安定化を高精度に実行するときに必要です。[RealSense D435iで撮影したキャリブレーション用のrosbag](https://www.dropbox.com/s/e4onsul90wjbsad/aruco_board_d435i.bag?dl=1)をダウンロードします。
以下のコマンドを別々のターミナルで実行してください。
```
$ roslaunch virtualgimbal_ros estimate_line_delay_d435i.launch  
$ rosbag play aruco_board_d435i.bag --clock -s 10 # rosbagの置かれたディレクトリで実行  
```
Line delayを推定する画面が起動します。回転するArUcoボードから、マーカを1個づつ個別に検出して、角度の変化を計算します。最後に例えば`Inlier:7999 / 10198 Line_delay:0.00003025 [second] `と表示されたらline delayの推定が完了です。ここでの値は毎回多少変化します。このline delayの値は後述するvirtualgimbal_ros_nodeのパラメータのline_delayに設定します。

# 3. Nodes
## 3.1 virtualgimbal_ros_node  
virtualgimbal_ros_node はカメラで撮影した動画とIMUで計測した角速度から、安定化した動画を生成します。
![nodes](https://github.com/yossato/images/blob/master/nodes.png?raw=true)  

### 3.1.1 Subscribe Topics
#### image_raw (sensor_msgs/Image)  
Rectified image stream from the camera driver.
  
#### camera_info (sensor_msgs/Camerainfo)  
Camera metadata.
  
#### imu_data (sensor_msgs/Imu)  
Angular velocity.  
  
## 3.1.2 Publish Topics
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
|offset_time|double|0|ジャイロセンサと画像ストリームのタイムスタンプのオフセット時間(秒)。synchronizer_nodeにより計測できる。カメラによって値が異なる。|
|line_delay|double|0.0|ローリングシャッターカメラのline delay(秒)。行ごとに露光タイミングの異なるローリングシャッターの画像センサーに関して、1行ごとの露光タイミングの遅延時間を表す。この値はline_delay_estimator_nodeで推定する。グローバルシャッターの画像センサーでは0に設定する。|
|verbose|bool|false|デバック機能を提供する。trueで各種ログが有効化され、安定化の様子がグラフで表示される。falseでログ機能を停止する。|
|allow_blue_space|bool|false|trueで画像に青い部分が生じることを許すと、ブレ補正能力が大幅に向上する。しかし、カメラのブレが大きくなりすぎて限界を迎えると画面の端に青い部分が生じる。falseにするとブレ補正能力が低下するが、画面端に青い部分が生じない。|
|lsm_period|double|1.5|VirtualGimbalの動画安定化アルゴリズムは、時系列方向に平滑化したカメラ角度と、平滑化していない生のカメラ角度の差の情報から、画像のシフト量を計算している。このときの時系列のカメラ角度を平滑化する方法としてVirtualGimbal ROSでは、最小二乗法を採用した。最小二乗法は任意の次数とフィッティングの対象とする時間幅を選択できる。このlsm_periodは、最小二乗法(least squares method)を計算する時間幅(秒)である。値を大きくすると、長時間のカメラの動きから平滑化したカメラ角度を計算するため、動画の安定化能力が向上するが、急なカメラの動きに追従できなくなる。値を小さくすると、短時間のカメラの動きをから平滑化したカメラ角度を計算するため、安定化能力が低下するがカメラの急な動きに追従できるようになる。|
|lsm_order|double|1|最小二乗法でフィッティングする曲線の次数。1だと1次式の直線でフィッティングする。2だと2次式の放物線でフィッティングする。次数を上げると追従性が向上するが安定化能力が低下する。1次式でフィッティングすると安定性は高まるが追従性が低い。|
  
## 3.2 synchronizer_node  
synchronizer_node は動画とIMU間のタイムスタンプのオフセットを精密測定するノード。一般的に、カメラの動画とIMUの角速度のタイムスタンプには僅かな時刻の差(オフセット)がある。動画の安定化にはミリ秒以下の精度の同期が必要になるため、このオフセットを正しく設定しないと動画の安定化品質が低下する。  
virtualgimbal_rosではカメラのシャッターの露光中心をタイムスタンプの基準として利用している。synchronizer_nodeはSum of Absolute Differences (SAD)により動画とIMUの角速度の相関を計算し最も良く相関があるオフセットの値を算出する。もう少し詳しく説明すると本ノードは、1.動画からオプティカルフローを計算、2.オプティカルフローから角速度を推定、3.推定された角速度とIMUにより計測された角速度の相関をオフセットを少しずつ変化させながら計算、という処理を実行する。相関が一番良いオフセットの部分で一番SADの値が小さくなるためオフセットを推定できる。得られたオフセットはstabilize.launchのparamのoffset_timeにセットして使用する。  
  
### 3.2.1 Subscribe Topics
#### image_raw (sensor_msgs/Image)  
Rectified image stream from the camera driver.
  
#### camera_info (sensor_msgs/Camerainfo)  
Camera metadata.
  
#### imu_data (sensor_msgs/Imu)  
Angular velocity.  
  
### 3.2.2 Parameters  
|Parameter|Type|default|description|
|:---|:---|:---|:---|
|image|string|image|入力画像トピック|
|imu_data|string|imu_data|入力角速度トピック|
|maximum_offset_time|float|0.5|SADを計算するオフセットの最大値(秒)。動画とIMUのタイムスタンプ差の最大値を指定してください。値を大きくすると動画とIMUのタイムスタンプの誤差が大きくてもオフセットの推定が可能ですが、計算に時間がかかるようになります。|
|correlation_time|float|15.0|SADを計算する時間の長さ(秒)。長くするとオフセットの推定精度が向上するが、推定に必要な動画の長さが長くなり、推定に時間がかかるようになる。|
    
## 3.3 line_delay_estimator_node
line_delay_estimator_nodeは、一般的なローリングシャッターのCMOSイメージセンサ有するLine delayを推定する。ローリングシャッターのCMOSイメージセンサは、撮像するときに1行毎読み込むために、得られる画像の各行で撮影タイミングが異なる。この1行ごとの撮影タイミングの差を本ノードは推定できる。このノードは画面にArUcoマーカのボードを表示する。line delayの推定を行うには、このArUcoマーカのボードを撮影しながらカメラを回転される。カメラを回転させると相対的にボードが回転する。この回転するボードのマーカを撮像すると行ごとに撮影タイミングが異なり、撮影した画像の上下で映るマーカの角度がわずかに変化する。角度の変化から1行ごとに生じるline delayを推定できる。line delayの単位は秒。実行後して十分なデータが取得できると最後に結果が表示される。
    
### 3.3.1 Subscribe Topics
#### image (sensor_msgs/Image)  
Rectified image stream from the camera driver.

#### camera_info (sensor_msgs/CameraInfo)
Camera metadata.

### 3.3.2 Parameters  
|Parameter|Type|default|description|
|:---|:---|:---|:---|
|image|string|image|入力画像トピック|
|minimum_angle_thresh|float|0.05|カメラを回転させたときに推定用データとして受け付けるフレーム間での最小の角度の差分[radian]のしきい値 。値を大きくするとカメラを高速で回転させる必要がある。|
|maximum_relative_delay_ransac|float|0.01|Line delayを推定するときに外れ値の影響を除去するRANSACアルゴリズムについて、inlierとしてみなす仮モデルによる予測値とサンプルの差分の大きさ。(0,1]で設定する。値を大きくすると外れ値に弱くなる。|
|maximum_iteration_ransac|float|10000|RANSACの最大反復回数。回数を増やすと計算に時間がかかるが結果が洗練される。|
|minimum_number_of_data_ransac|float|10000|RANSACに入力するデータの数。値を大きくすると推定される結果が安定するが、データ取得に時間がかかるようになる。|
|generate_aruco_board|bool|false|trueに設定すると印刷して使うためのPNG形式のArUcoボードを生成する。後述するmarker_params.yamlに書かれたArUcoボードのパラメータを変更したときなどはtrueに設定して、PNG画像を再生成すること。|
|show_gui|bool|true|falseに設定すると、キャリブレーション状況の確認用GUIウィンドウを表示しません。|

Line delay推定用のパラメータはparams/line_delay_estimation_params.yamlに記録してある。  
このノードはOpenCVのArUcoマーカを利用しているため、他にもArUcoマーカの検出用のパラメータが多数存在し、params/detector_params.yamlとparams/marker_params.yamlに保存してある。詳細は[Detection of ArUco Markers](https://docs.opencv.org/4.5.2/d5/dae/tutorial_aruco_detection.html)を参考のこと。これらのパラメータはROS launchで起動するときにrosparamコマンドにより読み込まれる。
  
    
# 4. Launch files
## 4.1 stabilize.launch
イメージストリームを安定化します。  

## 4.2 stabilize_realsense_rgb.launch
Intel RealSense D435iのRGBイメージストリームについてIMUの角速度を用いて安定化します。paramとしてD435iのトピック名を指定してstabilize.launchを起動します。画像サイズは1920x1080 pixelを想定しています。

## 4.3 stabilize_realsense_ir.launch
Intel RealSense D435iのirカメラの左側のイメージストリームについてIMUの角速度を用いて安定化します。paramとしてD435iのトピック名を指定してstabilize.launchを起動します。

## 4.4 synchronizer.launch
画像ストリームとIMUの時刻同期に必要なオフセット時間を推定します。  

## 4.5 synchronize_realsense_rgb.launch
Intel RealSense D435iのRGBイメージストリームとIMUの同期を取ります。  

## 4.6 synchronize_realsense_ir.launch
RGBと同様にIRイメージストリームについてもIMUとの同期を取ります。  


