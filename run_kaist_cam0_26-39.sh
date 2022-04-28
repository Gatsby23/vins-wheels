#!/bin/bash
pathWrite='/home/q_ftp/DataSet2/newdata/' #
yamlPath_38_39='/home/q/linux/code/vins_fusion_ws/src/vins-fusion/config/kaist/kaist_cam0_viwo_38-39.yaml' #Example, it is necesary to change it by the dataset path
yamlPath_18_37='/home/q/linux/code/vins_fusion_ws/src/vins-fusion/config/kaist/kaist_cam0_viwo_18-37.yaml' #Example, it is necesary to change it by the dataset path

#------------------------------------
echo "run cam_calib_data_2021-08-06-8--------------------------"
#./mono_uisee ../../Vocabulary/ORBvoc.txt Uisee.yaml /media/qcx/LENOVO/ALINUX/dataset/Uisee/cam_calib_data_2021-08-06/dump_images/

echo "run kaist39 ang5----------------------"
./kaist_viwo /home/q/linux/code/vins_fusion_ws/src/vins-fusion/config/kaist/kaist_cam0_viwo_39_ang5.yaml /home/q_ftp/DataSet1/KAIST/kaist39/ "$pathWrite"/39/ang5/

echo "run kaist38 ang5---------------------------"
./kaist_viwo /home/q/linux/code/vins_fusion_ws/src/vins-fusion/config/kaist/kaist_cam0_viwo_38_ang5.yaml /home/q_ftp/DataSet1/KAIST/kaist38/urban38-pankyo/ "$pathWrite"/38/ang5/

echo "run kaist28 ang5-----------------------------"
./kaist_viwo /home/q/linux/code/vins_fusion_ws/src/vins-fusion/config/kaist/kaist_cam0_viwo_28_ang5.yaml /home/q_ftp/DataSet1/KAIST/kaist28/urban28-pankyo/ "$pathWrite"/28/ang5/

echo "run kaist26 ang5------------------------------"
./kaist_viwo /home/q/linux/code/vins_fusion_ws/src/vins-fusion/config/kaist/kaist_cam0_viwo_26_ang5.yaml /home/q_ftp/DataSet1/KAIST/kaist26/urban26-dongtan/ "$pathWrite"/26/ang5/


echo "run kaist39 ang500----------------------"
./kaist_viwo /home/q/linux/code/vins_fusion_ws/src/vins-fusion/config/kaist/kaist_cam0_viwo_39.yaml /home/q_ftp/DataSet1/KAIST/kaist39/ "$pathWrite"/39/ang500/

echo "run kaist38 -ang500--------------------------"
./kaist_viwo /home/q/linux/code/vins_fusion_ws/src/vins-fusion/config/kaist/kaist_cam0_viwo_38.yaml /home/q_ftp/DataSet1/KAIST/kaist38/urban38-pankyo/ "$pathWrite"/38/ang500/

echo "run kaist28 ang500-----------------------------"
./kaist_viwo /home/q/linux/code/vins_fusion_ws/src/vins-fusion/config/kaist/kaist_cam0_viwo_28.yaml /home/q_ftp/DataSet1/KAIST/kaist28/urban28-pankyo/ "$pathWrite"/28/ang500/

echo "run kaist26 ang500------------------------------"
./kaist_viwo /home/q/linux/code/vins_fusion_ws/src/vins-fusion/config/kaist/kaist_cam0_viwo_26.yaml /home/q_ftp/DataSet1/KAIST/kaist26/urban26-dongtan/ "$pathWrite"/26/ang500/






