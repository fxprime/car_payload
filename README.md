<h1>SLAM</h1>

```
roslaunch car_connect car_2d_mapping.launch 
```

<h1>Localize</h1>

```
rosrun car_connect carto_init.py
roslaunch car_connect car_2d_localization.launch load_state_filename:=${HOME}/Desktop/online_map_save.pbstream 
```


<h1>save map</h1>

```
rosservice call /finish_trajectory 0

rosservice call /write_state "{filename: '${HOME}/Desktop/online_map_save.pbstream'}"

```


