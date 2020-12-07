        
camera.h里面的相机模型        
KANNALA_BRANDT,//k表示等距畸变系数 鱼眼相机模型，使用一个标准相机加一个面镜 KB模型有时候被作为针孔相机的畸变模型，同时也是opencv中使用的鱼眼相机模型，而在kalibr 中，被用作等距畸变模型（equidistant distortion model）.
MEI,
PINHOLE,//针孔相机
PINHOLE_FULL,
SCARAMUZZA
据大佬说，根据经验，小于90度使用Pinhole，大于90度使用MEI模型。