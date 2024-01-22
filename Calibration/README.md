<html lang="en"><head>
    <meta charset="UTF-8">
<body marginheight="0"><h1>1 Prerequisites</h1>
<h2>1.1 Ubuntu16.04(recommend)</h2>
<h2>1.2 ROS kenetic</h2>
<h2>1.3 Opencv3.X</h2>
<h2>1.4 pcl 1.X</h2>
<h2>1.5 ceres 1.X</h2>
<h1>2 Building Camera-Lidar-calibration</h1>
<p>Clone the repository:  
</p>
<pre><code>get clone https://github.com/SEU-LMD/Camera-Lidar-Calibration.git</code></pre>
<p>We provide a script build.sh to build the Thirdparty libraries and Camera-Lidar-Calibration. Please make sure you have installed all required dependencies. Execute:
</p>
<pre><code>cd Camera-Lidar-Calibration
chmod +x build.sh
./build.sh</code></pre>
<p>This will create libKITTICornerDetection.so at lib folder and the executables in Examples folder.
</p>
<h1>3 Data preparation</h1>
<p>We need to prepare images and point cloud data to be stored in corresponding folders 'data'
</p>
<pre><code>data/leftImg/left_xx.png
...
data/rightImg/left_xx.png
...
data/lidar/xx.txt
...
data/3D.txt
data/Config.txt
data/names.txt</code></pre>
<p><em>names.txt</em> file is used to store the corner coordinates of the calibration plate<br><em>3D.txt</em> file is used to store the corner coordinates of the calibration chessboards
</p>
<h1>4 Run</h1>
<pre><code>roslaunch camera_calibration start.launch</code></pre>
<p>Edit By <a href="http://mahua.jser.me">MaHua</a>
Edit By <a href="http://mahua.jser.me">MaHua</a></p>
</body></html>
