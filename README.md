# strl_robotics - STRL Robotics framework for mobile robots


**1. Система координат карты проходимости (occupancy grid), строящейся методом RTabMap** 
---
(Андрей Криштопик)<br/>
Occupancy grid в rtabmap представлена стандартными сообщениями ros. Подробнее про систему координат occupancy grid можно почитать в [google doc’е](https://docs.google.com/document/d/1c-a6FynTeuAUqqo1ZnpUuR1_c4Y5wkpEJcfqtCDtESY/edit?usp=sharing) в разделе “Работа с Occupancy Grid”. Также в папке occupancy\_grid\_mapping есть папка rtabmap\_example, в которой находится скрипт occupancy\_grid\_demo.py, реализующий работу с occupancy grid. Про этот скрипт написано в том же google doc'е в разделе "Реализация работы с occupancy grid".



**2. Переход из системы координат карты в систему координат одометрии (в систему координат baselink робота)** 
---
(Андрей Криштопик + Линар Абдразаков + Владислав Головин)<br/>
Чтобы получить трансформацию между различными системами координат, можно использовать функцию [lookupTransform](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28C%2B%2B%29) для C++ или [lookup_transform](http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20listener%20%28Python%29) для Python. Пример получения координат робота в мировой системе координат можно посмотреть в скрипте occupancy\_grid\_demo.py в папке occupancy\_grid\_mapping/rtabmap\_example/scripts в функции get\_robot\_world\_pose. 



**3. Перевод в систему координат base_link робота для одометрии.** 
---
(Линар Абдразаков)<br/>
Есть три основных систем координат:
- map - система координат карты;
- odom - система координат, относительно которой считается одометрия (совпадает с системой координат base_link при запуске одометрии);
- base_link - система координат робота.
<br/>
В сообщениях одометрии хранится трансформация между двумя системами координат (поля frame_id и child_frame_id). Каждый метод одометрии имеет свои системы координат frame_id и child_frame_id, информация о которых не публикуется в топик /tf. Поэтому их нужно привести к общим системам координат так, чтобы frame_id = odom и child_frame_id = base_link. 
Перевод одометрии в систему координат base_link происходит с использованием пакета [tf_transformer](odometry/odometry_fusion/tf_transformer), который автоматически запускается вместе с robot_localization.



**4. О запуске RTabMap и построения ocupancy grid в multisession режиме** 
---
(Андрей Криштопик)<br/>
В папке occupancy\_grid\_mapping есть папка rtabmap\_example, в которой находится launch файл для запуска rtabmap и вспомогательные скрипты. Про запуск rtabmap с помощью launch файла из rtabmap\_example написано в [google doc'е](https://docs.google.com/document/d/1CMNFhYlmfJb-XJJDk92J0y-mMTqQdyz_rtd2w-TJOmI/edit?usp=sharing). В этом же google doc'е описана работа в режиме multi-session.



**5. О запуске комплексирования данных одометрии** 
---
(Линар Абдразаков)<br/>
Запуск методов одометрии и их комплексирования описан [здесь.](odometry) <br/>
Комплексирование происходит трех видов одометрии: [визуальная](odometry/visual_odometry), [лидарная](odometry/lidar_odometry) и колесная. <br/>
Перед комплексированием каждая одометрия приводится к системе координат base_link (frame_id=odom, child_frame_id=base_link).
Изменить конфигурацию комплексирования можно в [данном файле](odometry/odometry_fusion/robot_localization/params/husky_mipt_odometry_fusion.yaml). Описание параметров конфигурации можно найти [здесь](http://docs.ros.org/en/melodic/api/robot_localization/html/state_estimation_nodes.html). 


**6. О планировании движения к цели, заданной на карте** 
---
(Владислав Головин)<br/>
Планирование движения осуществляется с помощью дополнительных узлов, находящихся в папке [planning](planning). Для постановки задачи планировщику используются как стандартные инструменты ROS, а именно, [rviz](http://wiki.ros.org/rviz) со встроенным в него пакетом [move\_base\_simple](http://wiki.ros.org/move_base_simple), так и вручную созданные узлы, передающие [карту проходимости](occupancy_grid_mapping) и [одометрию](odometry). Подробнее о работе узла планирования написано [здесь](planning/README.md).

**7. О реализации движения по траектории** 
---
(Мухаммад Алхаддад)<br/>
Узел управления принимает сообщению позиции от одометрии и путь от планировщика как вход и выдает линейная и угловая скорости. Узел управления проверит все пути от планировшика и параллельно отслеживает последний обновленный путь. Пропорциональный регулятор был использован и только точки x и y от планировщика. Также учитывали ограничения на скорости и ускорения.

**8. О программном управлении манипулятором** 
---
(Константин Миронов)<br/>

**9. Об обнаружении 3D-позы манипулируемых объектов (например, кнопок лифта)** 
---
(Сергей Линок)<br/>

**10. Важная информация о состоянии робота** 
---
(Линар Абдразаков)<br/>

Основная информация о состоянии робота публикуется в топик /status. 

**Пример вывода топика /status при питании робота от сети**

```
header:                              
  seq: 28287
  stamp:
    secs: 1614184608
    nsecs: 173392658
  frame_id: ''
uptime: 28330379
ros_control_loop_freq: 9.9974492508
mcu_and_user_port_current: 0.82
left_driver_current: 0.0
right_driver_current: 0.0
battery_voltage: 26.88
left_driver_voltage: 0.0
right_driver_voltage: 0.0
left_driver_temp: 0.0
right_driver_temp: 0.0
left_motor_temp: 0.0
right_motor_temp: 0.0
capacity_estimate: 480
charge_estimate: 1.0
timeout: False
lockout: True
e_stop: True
ros_pause: False
no_battery: True
current_limit: False
```
