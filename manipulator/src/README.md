# sKILL-bot project

### 1. [Objects manipulation](objects_manipulation_v2)

> Реализация скилов манипулятора в виде ROS механизмов service/action. \
> Готовые скилы:
> - `pick_up_object`
> - `put_object` (on floor, on table, on chair)
> - `put_in_drawer`
> - `put_in_box`
>
> Интегрированные в `behav tree`:
> - `pick_up_object`
> - `put_object` (on table, on chair)
>
> Также пакет занимается публикацией трансформации манипулятора в `tf-tree`: `ur_gripper`, `rs_camera`

### 2. [Aruco localizator](aruco_localization_v2)

>Пакет предоставляет ноду для определения 3Д позы aruco-маркера на основании карты глубин с камеры. Публикует 3Д координаты углов маркера в топик `/<nodename>/objects`. \
Ограничения:
> - Считывает только маркеры из словаря `6х6` 
> - Не использует топик `CameraInfo`
> - Сильно загружает одно ядро процессора при 30 fps

### 3. [Skillbot scripsts](skillbot_scripts)

> Пакет для хранения полезных скриптов слишком коротких для выделения в каждого в отдельный пакет. На данный момент содержит в себе: 
> - `dankmux` - скрипт для удобного создания необходимого числа окон в сессии в `tmux`
> - `topic_mirror` - дублирование топика под другим именем
    

## Requirements 

1. `ROS-1` (melodic/noetic)
2. Python 3.6-3.8
3. OpenCV

## Setup catkin workspace

```bash
mkdir skillbot_ws
cd skillbot_ws
mkdir src
catkin_build
cd src
git clone --recurse-submodules git@git.sberrobots.ru:mipt.navigation/control_arm/ur5_skillbot.git
```

## Build


### 1a. ROS Noetic

`NOTE:` может собираться не с первого и не со второго раза

`geometry2` commit [fe0457f](https://github.com/ros/geometry2/tree/fe0457fe0d761a1dcc9182f9cc09912b3a4a8e2d)

```bash
catkin_make -j1 --pkg aruco_localization_v2 --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so
source devel/setup.bash
```

### 1b. ROS Melodic

`geometry2` commit [d0338cd](https://github.com/ros/geometry2/tree/d0338cd4c43498a2d868058f10259efeeb1de744)

```bash
catkin_make -j1 --pkg aruco_localization_v2 --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so
source devel/setup.bash
```


## Launch

### a) Вручную

Первый терминал

```bash
source devel/setup.bash
rosrun objects_manipulation_v2 robo_cleaner.py
```

Второй терминал

```bash
source devel/setup.bash
roslaunch aruco_localization_v2 huskyRS.launch
```

#### Запуск [сегментатора](https://git.sberrobots.ru/mipt.navigation/object_pose_estimation/husky_tidy_bot_cv)


Третий терминал 

```bash
cd ~/krishtopik/husky_tidy_bot_cv_ws/src/husky_tidy_bot_cv/scripts
source ~/krishtopik/husky_tidy_bot_cv_ws/devel/setup.bash
source ~/krishtopik/cvbridge_build_ws/devel/setup.bash --extend
python3 yolov8_node.py -vis
```

Четвертый терминал. На текущий момент не всегда запускается с первого раза

```bash
cd ~/krishtopik/husky_tidy_bot_cv_ws/src/husky_tidy_bot_cv/scripts
source ~/Repos/manipulator/geom_ws/devel/setup.bash
source ~/krishtopik/husky_tidy_bot_cv_ws/devel/setup.bash --extend
source ~/krishtopik/cvbridge_build_ws/devel/setup.bash --extend
python3 object_pose_estimation_node.py -id 0 -align-90
```


### b) [Dankmux:ldcfg](skillbot_scripts/dankmux/ldcfg.py)

```bash
source devel/setup.bash
rosrun skillbot_scripts ldcfg.py skillbot.json
```

# Tests

```
catkin_make run_tests_objects_manipulation_v2
```


## Notebooks

### 1. [gripper fit](notebooks/gripper_fit.ipynb)

Эксперименты с оптимизацие позы захвата к объекту. Сцена представляет собой набор 3д объектов. Оптимизатор учитывает коллизии гриппера с различными объектами на сцене. 

### 2. [mesh to point to mesh](notebooks/mesh2p2m.ipynb)

**Не актуальный бейзлайн**, оставил для себя как небольшие заметки по работе с `trimesh` и `open3d`

### 3. [robot fit](notebooks/robot_fit.ipynb)

Эксперименты с оптимизацией позы робота с учетом рабочего пространства манипулятора. Оптимизатор `SGD` из `pytorch`. 

### 4. [robot fit final](notebooks/robot_fit_final.ipynb)

Логическое продолжение [robot fit](notebooks/robot_fit.ipynb): структуризация кода, существенно увеличена скорость работы. Имеется бенчмарк производительности. При измерении быстродействия было выявлено падение скорости от использования GPU. \
Увеличение скорости работы было достигнуто путем увеличения learning rate и уменьшения точности поиска локального минимума функции стоимости.

`TODO:` Добавить препятствия

