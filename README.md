# Dual-LiDAR 3D Object Detection for Construction Site

## Dual-LiDAR Point Cloud Fusion with Automatic Alignment for Enhanced 3D Object Detection in Construction Site Environments

#### [[Paper]](https://)

[[Chao He]](https://scholar.google.com/citations?user=g4Yv3BkAAAAJ&hl=en) and [[Da Hu]](https://scholar.google.com/citations?user=Y7_j-GMAAAAJ&hl=en&oi=ao) 

Kennesaw State University

This is the project page for [[Paper]](https://)

The proposed system employs two LiDAR sensors positioned 24 meters apart, with an automatic alignment pipeline combining Fast Point Feature Histogram (FPFH) descriptors and Iterative Closest Point (ICP) registration to precisely merge the two point clouds. The system performs coarse-to-fine alignment through RANSAC-based global registration followed by point-to-plane ICP refinement, achieving robust fusion without manual intervention. The merged point cloud provides enhanced spatial coverage and reduced occlusion compared to single-sensor configurations. We integrate this dual-LiDAR framework with our background filtering method to enable accurate 3D object detection of construction vehicle and personnel

<div align="center">
  <img src="1.png" width="100%" title="workflow"><br>
  <strong>Figure 1.</strong> Complete system workflow.
</div>
<br><br>


<div align="center">
  <img src="2.png" width="100%" title="workflow"><br>
  <strong>Figure 2.</strong> The setup of the experimental monitoring system.
</div>
<br><br>


<div align="center">
  <img src="3.png" width="100%" title="workflow"><br>
  <strong>Figure 3.</strong> Background filtering of point cloud frame.
</div>
<br><br>

<div align="center">
  <img src="4.png" width="100%" title="workflow"><br>
  <strong>Figure 4.</strong> Visualization of object detection. (green: worker, red: construction vehicle).
</div>
<br><br>

<div align="center">
  <img src="5.png" width="100%" title="workflow"><br>
  <strong>Figure 5.</strong> Workflow of muti-object tracking framework.
</div>
<br><br>

<div align="center">
  <img src="6.png" width="100%" title="workflow"><br>
  <strong>Figure 6.</strong> The visualization of the construction site in RViz.
</div>
<br><br>

<div align="center">
  <img src="6.png" width="100%" title="workflow"><br>
  <strong>Figure 6.</strong> The visualization of the construction site in RViz.
</div>
<br><br>


## 1. Dataset
**Dataset** : 
[[bin]](https://github.com/Saturn-Chao-He/Construction-Site-Tracking/tree/main/bin)

## 2. Environment (Ubuntu 20.04, ROS 1 Noetic)

Create Python environment and install the required packages:
```bash
conda env create -f track.yaml
conda activate track

```

## 3. Ternimal 1
Run
```bash
export DISABLE_ROS1_EOL_WARNINGS=1
source /opt/ros/noetic/setup.bash
roscore
```

## 4. Ternimal 2
Run
```bash
rviz -d tracking.rviz -f velodyne
```

## 5. VSCode
Run
```bash
# conda env: track
export DISABLE_ROS1_EOL_WARNINGS=1
source /opt/ros/noetic/setup.bash
python speed.py
```



## Acknowledgement
Great thanks to the Q building of Kennesaw State University.

## Cite
If this project is useful in your research, please cite:
> He, C., & Hu, D. (2025). A LiDAR-Driven Framework for Real-Time Monitoring and Speed Tracking on Construction Sites.

