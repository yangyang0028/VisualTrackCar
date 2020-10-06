import sensor, image, time
from pyb import UART, LED
import json
class PID:
    def __init__(self, limit, target, feedback, Kp, Ki, Kd, e_0, e_1, e_2):
        self.limit=limit
        self.target=target
        self.feedback=feedback
        self.Kp=Kp
        self.Ki=Ki
        self.Kd=Kd
        self.e_0=e_0
        self.e_1=e_1
        self.e_2=e_2
    def pid_calc(self):
        self.e_0 = self.target - self.feedback
        ep = self.e_0  - self.e_1
        ei = self.e_0
        ed = self.e_0 - 2*self.e_1 + self.e_2
        out = self.Kp*ep + self.Ki*ei + self.Kd*ed
        out = min(max(out, -self.limit), self.limit)
        self.e_2 = self.e_1
        self.e_1 = self.e_0
        return out
pid = PID(100, 0, 0,
    1.2, 0.002, 0,
    0, 0, 0)
led=LED(1)
led.on()
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()
uart = UART(3, 115200)
control_val=0
control_val=0
MIN_LIMIT=-300
MAX_LIMIT=300
ans_x=0
ans_y=160
Roi=[ans_x,ans_y,320,50]
aaa=28
while(True):
    quan=0;
    quan_id=-1
    img = sensor.snapshot(1.8)
    blobs = img.find_blobs([(5, 32, -21, 11, -14, 24)],roi=Roi,pixels_threshold = 24,area_threshold = 5,merge = True)
    img.draw_rectangle(Roi)
    for b in blobs:
        if quan<b.pixels():
            quan=b.pixels()
            quan_id=b
    if quan > 0:
        x = quan_id[0]
        y = quan_id[1]
        width = quan_id[2]
        height = quan_id[3]
        img.draw_rectangle([x,y,width,height])
        img.draw_cross(quan_id[5], quan_id[6])
        pid.feedback=x
        pid.target=160
        control_val += int(pid.pid_calc())
        control_val = max(min(control_val, MAX_LIMIT), MIN_LIMIT)
        print(control_val)
        control_val+=556
        uart.write(aaa.to_bytes(1,'int')+control_val.to_bytes(2,'int'))
        control_val-=556
