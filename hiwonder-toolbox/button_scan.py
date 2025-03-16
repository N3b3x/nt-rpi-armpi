#!/usr/bin/env python3
import gpiod
import os, time
key1_pin = 13
key2_pin = 23

def reset_wifi():
    os.system("sudo rm /etc/wifi/* -rf > /dev/null 2>&1")
    os.system("sudo systemctl restart wifi.service > /dev/null 2>&1")

if __name__ == "__main__":
    chip = gpiod.Chip('gpiochip4')
    key1 = chip.get_line(key1_pin)
    key1.request(consumer="key1", type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)
    key2 = chip.get_line(key2_pin)
    key2.request(consumer="key2", type=gpiod.LINE_REQ_DIR_IN, flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)

    key1_pressed = False
    key2_pressed = False
    servo_test = False
    count = 0
    while True:
        if key1.get_value() == 0:
            time.sleep(0.05)
            if key1.get_value() == 0:
                count += 1
                servo_test = True
                if count == 60 and not key1_pressed:
                    count = 0
                    key1_pressed = True
                    servo_test = False
                    print('reset_wifi')
                    reset_wifi()
            else:
                count = 0
                key1_pressed = False
                continue
            
        elif key2.get_value() == 0:
            time.sleep(0.05)
            if key2.get_value() == 0:
                count += 1
                if count == 60 and not key2_pressed:
                    count = 0
                    key2_pressed = True
                    print('sudo halt')
                    os.system('sudo halt')
            else:
                count = 0
                key1_pressed = False
                continue
        else:
            if servo_test:
                servo_test = False
                os.system("python3 /home/pi/ArmPi_mini/armpi_mini_demo/hardware_test.py")
            key1_pressed = False
            key2_pressed = False
            count = 0
            time.sleep(0.05)

        
