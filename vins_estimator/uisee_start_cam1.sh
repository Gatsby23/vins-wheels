#!/bin/bash

#------------------------------------
echo "run cam_calib_data_2021-08-06-8--------------------------"
#./mono_uisee ../../Vocabulary/ORBvoc.txt Uisee.yaml /media/qcx/LENOVO/ALINUX/dataset/Uisee/cam_calib_data_2021-08-06/dump_images/
echo "run car23_0812_imu_dump_gps-8----------------------"
/home/qcx/linux/code/vins_fusion_no_sfm_ws/build/devel/lib/vins/uisee_viwo /home/qcx/linux/code/vins_fusion_no_sfm_ws/src/vins-fusion/config/uisee/uisee_cam1_sfm_config.yaml /media/qcx/LENOVO/ALINUX/dataset/Uisee/car23_0812_imu_dump_gps/

echo "run log_2021-08-26-10-35-57-square ---------------------------"
/home/qcx/linux/code/vins_fusion_no_sfm_ws/build/devel/lib/vins/uisee_viwo /home/qcx/linux/code/vins_fusion_no_sfm_ws/src/vins-fusion/config/uisee/uisee_cam1_sfm_config.yaml /media/qcx/LENOVO/ALINUX/dataset/Uisee/log_2021-08-26-10-35-57/dump_images/

echo "run log_2021-08-26-10-50-01-squareAndUp-----------------------------"
/home/qcx/linux/code/vins_fusion_no_sfm_ws/build/devel/lib/vins/uisee_viwo /home/qcx/linux/code/vins_fusion_no_sfm_ws/src/vins-fusion/config/uisee/uisee_cam1_sfm_config.yaml /media/qcx/LENOVO/ALINUX/dataset/Uisee/log_2021-08-26-10-50-01/dump_images/

echo "run log_2021-08-26-11-38-40-L------------------------------"
/home/qcx/linux/code/vins_fusion_no_sfm_ws/build/devel/lib/vins/uisee_viwo /home/qcx/linux/code/vins_fusion_no_sfm_ws/src/vins-fusion/config/uisee/uisee_cam1_sfm_config.yaml /media/qcx/LENOVO/ALINUX/dataset/Uisee/log_2021-08-26-10-35-57/dump_images/

echo "run log_2021-09-01-10-40-35-8--------------------------------"
/home/qcx/linux/code/vins_fusion_no_sfm_ws/build/devel/lib/vins/uisee_viwo /home/qcx/linux/code/vins_fusion_no_sfm_ws/src/vins-fusion/config/uisee/uisee_cam1_sfm_config.yaml /media/qcx/LENOVO/ALINUX/dataset/Uisee/log_2021-09-01-10-40-35/dump_images/

echo "run log_2021-09-01-10-46-28-s--------------------------------"
/home/qcx/linux/code/vins_fusion_no_sfm_ws/build/devel/lib/vins/uisee_viwo /home/qcx/linux/code/vins_fusion_no_sfm_ws/src/vins-fusion/config/uisee/uisee_cam1_sfm_config.yaml /media/qcx/LENOVO/ALINUX/dataset/Uisee/log_2021-09-01-10-46-28/dump_images/

echo "run log_2021-09-01-10-55-19-8------------------------------------"
/home/qcx/linux/code/vins_fusion_no_sfm_ws/build/devel/lib/vins/uisee_viwo /home/qcx/linux/code/vins_fusion_no_sfm_ws/src/vins-fusion/config/uisee/uisee_cam1_sfm_config.yaml /media/qcx/LENOVO/ALINUX/dataset/Uisee/log_2021-09-01-10-55-19/dump_images/

echo "run L1-----------------------------------------"
/home/qcx/linux/code/vins_fusion_no_sfm_ws/build/devel/lib/vins/uisee_viwo /home/qcx/linux/code/vins_fusion_no_sfm_ws/src/vins-fusion/config/uisee/uisee_cam1_sfm_config.yaml /media/qcx/LENOVO/ALINUX/dataset/Uisee/20200804_car23_imu_can_camera_yahui/L1/dump_images/

echo "run biaoding------------------------------------------"
/home/qcx/linux/code/vins_fusion_no_sfm_ws/build/devel/lib/vins/uisee_viwo /home/qcx/linux/code/vins_fusion_no_sfm_ws/src/vins-fusion/config/uisee/uisee_cam1_sfm_config.yaml /media/qcx/LENOVO/ALINUX/dataset/Uisee/20200804_car23_imu_can_camera_yahui/biaoding/dump_images/

#/media/qcx/LENOVO/ALINUX/dataset/Uisee/car23_0812_imu_dump_gps