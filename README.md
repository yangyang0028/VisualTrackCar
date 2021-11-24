# VisualTrackCar
 基于 OpenMV 和 STM32C8T6 的 循迹小车

原理
选取图片中部靠下的长方形区域为阈值化的ROI区域，

读到黑线的中心位置 做PID运算，将PID运算结果通过传到STM32上

通信协议如下

标志位 小车控制高八位 小车控制低八位

STM32上将OpenMV的PID计算结果进行平方处理并于基准速度相加，得到小车的占空比，并输出

![ls](image/t1.gif)
