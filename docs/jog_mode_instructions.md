# ArmPi Mini Jog Mode Guide

## Objective
To manually control the robotic arm using keyboard inputs for precise positioning, testing, and manual operation. Jog mode allows fine-grained control over each joint of the arm.

## Required Items
* Robotic arm properly connected and powered
* Camera system operational
* Clear workspace around the arm

---

## Step 1: Start the Calibration Script
First, run the script from your terminal:
```bash
python3 imx477/lab_adjust.py
```
Two windows will appear: 'Original' (the live camera feed) and 'Mask' (the color detection view). Make sure the 'Original' window is active to register your key presses.

---

## Step 2: Access Jog Mode (Key: `j`)
1. Press the `j` key to enter Jog Mode.
2. The camera will switch to high resolution (1920x1080) for better visibility.
3. A window titled "Jog Mode" will appear with live camera feed and control instructions.

---

## Step 3: Understanding the Controls
The jog mode provides real-time control over each joint of the arm:

### Arm Joint Controls
* **[f/g]** - **Gripper**: Open/close the gripper
  - `f`: Open gripper (increase position value)
  - `g`: Close gripper (decrease position value)

* **[q/e]** - **Elbow**: Move elbow joint up/down
  - `q`: Elbow up (increase angle)
  - `e`: Elbow down (decrease angle)

* **[a/d]** - **Shoulder**: Move shoulder joint forward/back
  - `a`: Shoulder forward (decrease angle)
  - `d`: Shoulder back (increase angle)

* **[w/s]** - **Lift**: Move lift joint up/down
  - `w`: Lift up (decrease angle)
  - `s`: Lift down (increase angle)

* **[z/c]** - **Base**: Rotate base left/right
  - `z`: Base rotate left (decrease angle)
  - `c`: Base rotate right (increase angle)

### Utility Controls
* **[r]** - **Reset**: Reset all joints to center position
* **[u]** - **Undistort**: Toggle camera undistortion on/off
* **[j]** - **Exit**: Exit Jog Mode and return to main menu

---

## Step 4: Using Jog Mode
1. **Start position**: The arm will automatically reset to center position when entering jog mode.
2. **Real-time feedback**: The current position values for each joint are displayed on screen.
3. **Fine control**: Each key press moves the joint by a small increment (2 degrees for joints, 50 units for gripper).
4. **Safety**: Move slowly and watch the arm to avoid collisions.

---

## Step 5: Best Practices
* **Start slow**: Use small movements to understand how each joint affects the arm position.
* **Watch the camera**: Use the camera feed to see the arm's position relative to objects.
* **Check workspace**: Ensure there are no obstacles before moving the arm.
* **Use reset**: Press `r` to return to a safe center position if needed.
* **Undistortion**: Toggle undistortion with `u` if you have camera calibration data for better accuracy.

---

## Step 6: Exiting Jog Mode
1. Press `j` to exit Jog Mode.
2. The camera will switch back to the original resolution.
3. You'll return to the main menu.

---

## Important Notes
* **Movement increments**: Each key press moves joints by 2 degrees, gripper by 50 units.
* **Real-time display**: Current joint angles and gripper position are shown on screen.
* **Safety first**: Always be aware of the arm's position and potential obstacles.
* **Resolution**: Jog mode uses high resolution (1920x1080) for better visibility.
* **Undistortion**: Can be toggled on/off during jog mode if camera calibration data exists.

---

## Troubleshooting
* **Arm not responding**: Check power and servo connections.
* **Erratic movement**: Ensure the arm is properly initialized and calibrated.
* **Poor visibility**: Toggle undistortion or adjust lighting.
* **Arm hitting limits**: Use the reset function (`r`) to return to center position.

---

## What Jog Mode Does
Jog mode provides:
1. **Manual control**: Direct keyboard control over each arm joint
2. **Real-time feedback**: Live display of joint positions and angles
3. **Precise positioning**: Fine-grained control for testing and calibration
4. **Safety features**: Reset function and controlled movement increments
5. **Visual feedback**: Live camera feed with position information overlay

Jog mode is essential for:
* Testing arm functionality
* Manual positioning for calibration
* Debugging arm movements
* Learning how each joint affects the arm's position
* Precise positioning for specific tasks

---

## Coordinate System
The arm uses the following coordinate system:
* **Base rotation**: Positive angles rotate right, negative angles rotate left
* **Lift**: Positive angles move down, negative angles move up
* **Shoulder**: Positive angles move back, negative angles move forward
* **Elbow**: Positive angles move down, negative angles move up
* **Gripper**: Higher values open the gripper, lower values close it

This mode is particularly useful for the 3D calibration process where you need to precisely position the arm at specific coordinates. 