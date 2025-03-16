#关掉开机自启
sudo systemctl stop armpi_mini.service

#重新开启开机自启
sudo systemctl restart armpi_mini.service

#测试上位机
python3 /home/pi/ArmPi_mini/armpi_mini_software/ArmPi_mini.py

# 测试机械臂
python3 /home/pi/ArmPi_mini/armpi_mini_demo/hardware_test.py
或者按下扩展板的Key1按键

# 软关机
# 长按Key2按键


# 第5章 机械臂基础运动课程
# 第3课 控制机械臂上下移动
python3  /home/pi/ArmPi_mini/armpi_mini_demo/arm_move_IK_demo.py
# 第4课 机械臂三轴联动
python3  /home/pi/ArmPi_mini/armpi_mini_demo/arm_move_IK_triaxial.py


# 第6章 AI视觉学习课程
# 第1课 颜色识别
python3 /home/pi/ArmPi_mini/functions/color_detect.py

# 第2课 颜色分拣
python3 /home/pi/ArmPi_mini/functions/color_sorting.py

# 第3课 目标位置检测
python3 /home/pi/ArmPi_mini/functions/position_detection.py

# 第4课 目标追踪
python3 /home/pi/ArmPi_mini/functions/color_tracking.py

# 第5课 智能码垛
python3 /home/pi/ArmPi_mini/functions/color_palletizing.py


# 第7章 上位机动作编辑课程
# 第5课 通过命令行的形式调用动作组（选看）
python3  /home/pi/ArmPi_mini/armpi_mini_demo/action_groups_control_demo.py

