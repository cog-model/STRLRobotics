# Aruco recognition with localization

Распознование aruco-маркеров методами `opencv` с их локализацией по карте глубин.

```
px, py - координаты в пикселях
depth[px, py] - соответственно глубина
x = depth[px, py]*(px - cx)/fx
y = depth[px, py]*(py - cy)/fy
z = depth[px, py]
```
Камеры глубины сильно шумят, за глубину стоит брать среднюю глубину с квадрата $\pm 3$ пикселя (по задаче подбирается), при этом отбрасывая нулевые значения (в них камера глубины просто не знает глубину). \
Еще замечание: реалсенс выдает глубину в миллиметрах.

## Project structure

```
aruco_localization_v2/
├── include
│   ├── ArucoLocalizer.h
│   └── ImageConverter.h
├── launch
│   ├── d400.launch
│   └── huskyRS.launch
├── msg
│   ├── aruco_array_msg.msg
│   └── aruco_msg.msg
├── src
│   ├── aruco_local.cpp
│   ├── ArucoLocalizer.cpp
│   └── ImageConverter.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Launch

1.Для враппера реалсенса со стандартным конфигом

```bash
roslaunch aruco_localization_v2 d400.launch
```

2. Для запуска на Husky

```bash
roslaunch aruco_localization_v2 huskyRS.launch
```
Пример для запуска в фоне

```bash
roslaunch aruco_localization_v2 huskyRS.launch bkg:=true &
```

`TODO:` Добавить получчение калибровки камеры из топиков `CameraInfo`