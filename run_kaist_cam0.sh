#!/bin/bash
pathWrite='/home/qcx/linux/output/20220301' #
yamlPath_38_39='/home/q/linux/code/vins_fusion_ws/src/vins-fusion/config/kaist/kaist_cam0_viwo_38-39.yaml' #Example, it is necesary to change it by the dataset path
yamlPath_18_37='/home/q/linux/code/vins_fusion_ws/src/vins-fusion/config/kaist/kaist_cam0_viwo_18-37.yaml' #Example, it is necesary to change it by the dataset path

#------------------------------------
echo "run cam_calib_data_2021-08-06-8--------------------------"
#./mono_uisee ../../Vocabulary/ORBvoc.txt Uisee.yaml /media/qcx/LENOVO/ALINUX/dataset/Uisee/cam_calib_data_2021-08-06/dump_images/

#echo "run kaist39----------------------"
#./kaist_viwo "$yamlPath_38_39" /home/q_ftp/DataSet1/KAIST/kaist39/ "$pathWrite"/kaist39/
#
#echo "run kaist38 ---------------------------"
#./kaist_viwo "$yamlPath_38_39" /home/q_ftp/DataSet1/KAIST/kaist38/urban38-pankyo/ "$pathWrite"/kaist38/
#
echo "run kaist28-----------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist28/urban28-pankyo/ "$pathWrite"/kaist28/

echo "run kaist26------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist26/urban26-dongtan/ "$pathWrite"/kaist26/

echo "---------------------run kaist18------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist18/urban18-highway/ "$pathWrite"/kaist18/

echo "---------------------run kaist19------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist19/urban19-highway/ "$pathWrite"/kaist19/

echo "---------------------run kaist25------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist25/urban25-highway/ "$pathWrite"/kaist25/

echo "---------------------run kaist26------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist26/urban26-dongtan/ "$pathWrite"/kaist26/

echo "---------------------run kaist27------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist27/urban27-dongtan/ "$pathWrite"/kaist27/

echo "---------------------run kaist28------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist28/urban28-pankyo/ "$pathWrite"/kaist28/

echo "---------------------run kaist29------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist29/urban29-pankyo/ "$pathWrite"/kaist29/

echo "---------------------run kaist30------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist30/urban30-gangnam/ "$pathWrite"/kaist30/

echo "---------------------run kaist31------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist31/urban31-gangnam/ "$pathWrite"/kaist31/

echo "---------------------run kaist32------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist32/urban32-yeouido/ "$pathWrite"/kaist32/

echo "---------------------run kaist33------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist33/urban33-yeouido/ "$pathWrite"/kaist33/

echo "---------------------run kaist34------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist34/urban34-yeouido/ "$pathWrite"/kaist34/

echo "---------------------run kaist35------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist35/urban35-seoul/ "$pathWrite"/kaist35/

echo "---------------------run kaist36------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist36/urban36-seoul/ "$pathWrite"/kaist36/

echo "---------------------run kaist37------------------------------"
./kaist_viwo "$yamlPath_18_37" /home/q_ftp/DataSet1/KAIST/kaist37/urban37-seoul/ "$pathWrite"/kaist37/







