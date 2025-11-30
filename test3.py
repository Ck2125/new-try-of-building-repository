import RPi.GPIO as gpio
import time
from picamera2 import Picamera2
import cv2
import numpy as np
 
gpio.setmode(gpio.BCM) 

# pwm初始化
def pwm_init(dc):
    global pwm1, pwm2
    pwm1.start(dc)
    pwm2.start(dc)
   
# pwm改占空比
def pwm_change_dc(dc):
    global pwm1, pwm2
    pwm1.ChangeDutyCycle(dc)
    pwm2.ChangeDutyCycle(dc)

# 停止发出pwm
def pwm_stop():
    global pwm1, pwm2
    pwm1.stop()
    pwm2.stop()
    
# 根据不同mode选择占空比
def pwm_action(mode, is_init):
    if mode == "stop":
        dc = 0
    elif mode == "full":
        dc = 100
    elif mode == "low":
        dc = 30
    elif mode == "medium":
        dc = 50
    elif mode == "high":
        dc = 75
    else:
        return

    if is_init:
        pwm_init(dc)
    else:
        pwm_change_dc(dc)


pin1 = 5    
pin2 = 6
pin3 = 13
pin4 = 19

pin5 = 20
pin6 = 21

# 1-4是控制电机，5-6是输出pwm控制速度
# 这个电机比较特别，左边两轮子是一起动，右边两轮子是一起动
# 1是控制左边正转，2是控制左边反转
# 3是控制右边正转，4是控制右边反转
# 所以1HIGH2LOW，左边正转
# 3HIGH4LOW，右边正转
# 若12或34同HIGH或同LOW，左边或右边停转
gpio.setup(pin1, gpio.OUT)
gpio.setup(pin2, gpio.OUT)
gpio.setup(pin3, gpio.OUT)
gpio.setup(pin4, gpio.OUT)
gpio.setup(pin5, gpio.OUT)
gpio.setup(pin6, gpio.OUT)

pwm1 = gpio.PWM(pin5, 500)
pwm2 = gpio.PWM(pin6, 500)

# modes = ["stop", "full", "low", "medium", "high"]
mode = "low"
pwm_action(mode, True)

# 摄像头的基本配置，size是控制分辨率
picam2 = Picamera2()
# config = picam2.create_still_configuration()
config = picam2.create_still_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

def car_forward():
    gpio.output(pin1, gpio.HIGH)
    gpio.output(pin2, gpio.LOW)
    gpio.output(pin3, gpio.HIGH)
    gpio.output(pin4, gpio.LOW)

def car_backward():
    gpio.output(pin1, gpio.LOW)
    gpio.output(pin2, gpio.HIGH)
    gpio.output(pin3, gpio.LOW)
    gpio.output(pin4, gpio.HIGH)

def car_stop():
    gpio.output(pin1, gpio.LOW)
    gpio.output(pin2, gpio.LOW)
    gpio.output(pin3, gpio.LOW)
    gpio.output(pin4, gpio.LOW)
    
def car_turnaround():
    gpio.output(pin1, gpio.HIGH)
    gpio.output(pin2, gpio.LOW)
    gpio.output(pin3, gpio.LOW)
    gpio.output(pin4, gpio.HIGH)

def car_test():
    return
    
def get_max_dc(mode):
    if mode == "stop":
        return 0
    elif mode == "full":
        return 100
    elif mode == "high":
        return 75
    elif mode == "medium":
        return 50
    elif mode == "low":
        return 30
    return 50

# 控制转向（还不太完善）
def update_direction(direction, mode):
    base = get_max_dc(mode)
    max_dc = min(base * 1.2, 100)
    min_dc = max(base * 0.3, 0)
    diff = min(abs(direction) * 0.2, max_dc * 0.7)
    print(f"direction:{direction}, diff:{diff}")
    if direction > 20:
        p1, p2 = min(base + diff, max_dc), max(base - diff, min_dc)
    elif direction < -20:
        p1, p2 = max(base - diff, min_dc), min(base + diff, max_dc)
    else:
        p1 = p2 = base
        
    pwm1.ChangeDutyCycle(p1)
    pwm2.ChangeDutyCycle(p2)
    print(f"p1:{p1}, p2:{p2}")

car_stop()
car_forward()
while True:
    # print("go on...")

    # 获取图像，下面那个RGB->BGR不能少
    # 不然得到的颜色不正常
    frame = picam2.capture_array()
    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    # cv2.imshow("frame", frame)
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('gray', gray)  
    
    retval, dst = cv2.threshold(gray, 0, 255, cv2.THRESH_OTSU)
    
    dst = cv2.dilate(dst, None, iterations=2)
    # cv2.imshow('dilate_frame', dst)  

    dst = cv2.erode(dst, None, iterations=6)
    cv2.imshow('erode_frame', dst)
    
    row = dst[420]  

    black_idx = np.where(row == 0)[0]

    if len(black_idx) == 0:
        print("do not detect black")
        continue
    segments = []
    start = black_idx[0]

    for i in range(1, len(black_idx)):
        if black_idx[i] != black_idx[i-1] + 1:
            segments.append((start, black_idx[i-1]))
            start = black_idx[i]
    
    segments.append((start, black_idx[-1]))
    
    # print(f"row:{row}")
    # print(f"black_idx:{black_idx}")
    print(f"segments:{segments}")
    if row[0] == 0: 
        if len(segments) > 1:
            last_seg = segments[1]
        else:
            print("no find second black block")
            continue
    else:
        last_seg = segments[0]
            
    
    black_start, black_end = last_seg
    print(f"road is {last_seg}")
    
    center = (black_start + black_end) // 2
    direction = center - 320
    # print(f"center:{center}")
    # print(f"direction:{direction}")
    
    # panduan logic
    
    update_direction(direction, mode)
    

    if cv2.waitKey(1) & 0xFF == ord('q'):
        car_stop()
        break

picam2.stop()
cv2.destroyAllWindows()

    
