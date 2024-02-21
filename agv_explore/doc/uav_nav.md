## Mode1: SLAM Livox VIM4

### Branches:
- fast_lio_localization: git@github.com:tongtybj/FAST_LIO_LOCALIZATION.git develop/livox_mid360
- aerial_robot: git@github.com:tongtybj/aerial_robot.git develop/mini_quadrotor/livox_mid360


### Map Localiztion

#### only fast_lio_localization Mode

$ roslaunch fast_lio_localization localization_mid360.launch map:=`rospack find fast_lio`/PCD/8-326_scans.pcd livox_driver:=true headless:=true

#### UAV Mode (should berosmater)

- step1: Bringup

$ roslaunch mini_quadrotor bringup.launch

- step2: Fast lio localization

$ roslaunch fast_lio_localization localization_mid360.launch map:=`rospack find fast_lio`/PCD/8-326_scans.pcd headless:=true mapping:=false prefix:=quadrotor reverse_tf:=true

