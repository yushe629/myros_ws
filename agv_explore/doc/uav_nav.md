## Mode1: SLAM Livox VIM4
### fast lio localiztion

$ roslaunch fast_lio_localization localization_mid360.launch map:=`rospack find fast_lio`/PCD/8-326_scans.pcd livox_driver:=true headless:=true

## UAV (rosmater)

### Mode2: Bringup

$ roslaunch mini_quadrotor bringup.launch

### Fast lio localization

$ roslaunch fast_lio_localization localization_mid360.launch map:=`rospack find fast_lio`/PCD/8-326_scans.pcd headless:=true mapping:=false prefix:=quadrotor reverse_tf:=true

