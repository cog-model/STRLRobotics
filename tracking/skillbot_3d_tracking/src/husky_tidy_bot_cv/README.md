**Installation:**

* Install dependencies:
```
# subprocess-tee
cd ~
git clone http://github.com/andrey1908/subprocess-tee
cd ~/subprocess-tee
pip install .

```

* Build from container:
```
source /opt/ros/noetic/setup.bash 
cd tracking/skillbot_3d_tracking
catkin_make_isolated -DCMAKE_BUILD_TYPE=Release
```

---

**Running:**

Download [objects models](https://disk.yandex.ru/d/xJwhvkl7fE4gZA) to ```~/tracking/skillbot_3d_tracking/src/husky_tidy_bot_cv/objects_models```.

*Optional:* To run YOLOv8 segmentation for toy boxes, download model [config and weights](https://disk.yandex.ru/d/XSlWRtN0e4qprA) to ```~/tracking/skillbot_3d_tracking/src/husky_tidy_bot_cv/yolov8_weights``` folder.

To run nodes use corresponding scripts from ```bin``` folder.

0. Change cameras configuration:

```
bash bin/config_cameras.sh
```

1. Segmentation YOLOv8 (for testing purposes)

```
bash bin/yolov8_node.sh
```

2. 2D-Tracking

```
bash bin/bot_sort_node.sh
```

3. Point Cloud extraction

```
bash bin/object_point_cloud_extraction_node.sh
```

4. 3D-Tracking (needs cartographer to work)

```
bash bin/tracker_3d_node.sh
```

5. Object pose estimation

```
bash bin/object_pose_estimation_node.sh
```