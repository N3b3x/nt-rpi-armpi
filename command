# Disable auto-start
sudo systemctl stop armpi_mini.service

# Re-enable auto-start
sudo systemctl restart armpi_mini.service

# Test upper computer
python3 /home/pi/ArmPi_mini/armpi_mini_software/ArmPi_mini.py

# Test robotic arm
python3 /home/pi/ArmPi_mini/armpi_mini_demo/hardware_test.py
Or press Key1 on the expansion board

# Soft shutdown
# Long press Key2


# Chapter 5 Basic Robotic Arm Motion Course
# Lesson 3 Control robotic arm up and down movement
python3  /home/pi/ArmPi_mini/armpi_mini_demo/arm_move_IK_demo.py
# Lesson 4 Robotic arm three-axis linkage
python3  /home/pi/ArmPi_mini/armpi_mini_demo/arm_move_IK_triaxial.py


# Chapter 6 AI Vision Learning Course
# Lesson 1 Color recognition
python3 /home/pi/ArmPi_mini/functions/color_detect.py

# Lesson 2 Color sorting
python3 /home/pi/ArmPi_mini/functions/color_sorting.py

# Lesson 3 Target position detection
python3 /home/pi/ArmPi_mini/functions/position_detection.py

# Lesson 4 Target tracking
python3 /home/pi/ArmPi_mini/functions/color_tracking.py

# Lesson 5 Intelligent palletizing
python3 /home/pi/ArmPi_mini/functions/color_palletizing.py


# Chapter 7 Upper Computer Action Editing Course
# Lesson 5 Call action groups through command line (optional)
python3  /home/pi/ArmPi_mini/armpi_mini_demo/action_groups_control_demo.py

