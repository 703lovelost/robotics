# botbot_lidar

Пакет запускает Gazebo Sim (Harmonic) с миром `gpu_lidar_sensor.sdf`, спавнит URDF-робота и добавляет GPU-лидар, который публикует данные в Gazebo-топик `/scan`. Далее `ros_gz_bridge` пробрасывает `/scan` в ROS 2 как `sensor_msgs/LaserScan`, а RViz сразу открывается с настроенным отображением.

## Запуск

```bash
colcon build --packages-select botbot_lidar
. install/setup.bash
ros2 launch botbot_lidar botbot_lidar.launch.py
```

## Проверка

```bash
ros2 topic echo /scan
```
