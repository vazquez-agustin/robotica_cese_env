### R2B lounge

## Crear paquete

```bash
ros2 pkg create --build-type ament_cmake ramirez_vazquez_pkg
```

## Descargar rosbag

Dentro de la carpeta src/ramirez_vazquez_pkg ejecutar los siguientes comandos:

```bash
$ wget --content-disposition 'https://api.ngc.nvidia.com/v2/resources/org/nvidia/team/isaac/r2bdataset2023/3/files?redirect=true&path=r2b_lounge/metadata.yaml' -o 'r2b_lounge/metadata.yaml'

$ wget --content-disposition 'https://api.ngc.nvidia.com/v2/resources/org/nvidia/team/isaac/r2bdataset2023/3/files?redirect=true&path=r2b_lounge/r2b_lounge_0.db3' -o 'r2b_lounge/r2b_lounge_0.db3'
```

## Comandos manuales

```bash
$ ros2 bag play src/ramirez_vazquez_pkg/r2b_lounge/ -r2.0 --loop

$ ros2 run rqt_gui rqt_gui

$ ros2 run rviz2 rviz2
```

## Ejecución automática

```bash
$ colcon build --packages-select ramirez_vazquez_pkg

$ source install/setup.bash 

$ ros2 launch ramirez_vazquez_pkg play.py
```