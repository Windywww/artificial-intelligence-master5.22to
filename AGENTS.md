### 比赛规则
第二十一届全国大学生智能汽车竞赛 人工智能视觉组
比赛规则参考说明：
https://zhuoqing.blog.csdn.net/article/details/154697884?spm=1011.2415.3001.5331
https://zhuoqing.blog.csdn.net/article/details/157686751?spm=1011.2415.3001.5331
### 项目所用硬件
openART plus 2个: 采用micropython
分别运行openART\map_detect.py (全局视角地图识别) 和 openART\classification.py（第一视角分类识别）
i.MX RT1064 主板及核心板 1个
开源库路径："D:\college\718\seekfree\RT1064_Library"
运动控制传感器为1个imu660RB六轴传感器、四个1024线正交编码器。
### 项目结构
openART\map_detect.py ：识别全局第三人称视角的openart上运行的程序
openART\classification.py：第一人称箱子和目的地分类的程序
不读取 openART\ 中除二者之外的其他文件。
artificial-intelligence-vision-master\SeekFree_RT1064_Opensource_Library:在小车rt1064核心板运行的版本，其中我们写的代码在SeekFree_RT1064_Opensource_Library\project\user下。
不读取project\user\algorithm_src\sokoban_lut.c