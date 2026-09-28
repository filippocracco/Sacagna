import time

import RPi.GPIO as GPIO

from seriale import SERIALEPICO
from finecorsa import FINECORSA
from giroscopio import BNO055
from colore import APDS9960

class PID:
    def __init__(self):
        self.last_time = time.time()
        self.last_error = 0
        self.integral = 0
        
        self.Kp = 0.05 #0.08
        self.Ki = 0.2
        self.Kd = 0.0005
        
        self.limP = 2.5
        self.limI = 0.5
        self.limD = 1.5  
    
    def reset(self):
        self.last_time = time.time()
        self.last_error = 0
        self.integral = 0        
    
    def calcola(self, ang, vbase = 2.0, offset = 0):
        current_time = time.time()
        dt = current_time - self.last_time

        error = offset - ang

        self.integral += error * dt
        derivative = (error - self.last_error) / dt
        
        P = self.Kp * error
        I = self.Ki * self.integral
        D = self.Kd * derivative
        
        if P > self.limP:
            P = self.limP
        if P < -self.limP:
            P = -self.limP
        if I > self.limI:
            I = self.limI
        if I < -self.limI:
            I = -self.limI
        if D > self.limD:
            D = self.limD
        if D < -self.limD:
            D = -self.limD
        
        out = P + I + D

        self.last_time = current_time
        self.last_error = error
        
        dx = vbase + out
        sx = vbase - out

        return dx, sx

class MOVIMENTI:
    
    def __init__(self):
        self.ser = SERIALEPICO()
        self.fc  = FINECORSA()
        self.bno = BNO055()
        self.apds = APDS9960()
        self.pid = PID()
        self.apds_ok = False

    def avanti(self, cm = 26.5, nero = 700, blu = 300, centro = 0, finale = 0):
        
        if not self.apds_ok: # l'APDS basta configurarlo una volta
            self.apds.begin()
            self.apds_ok = True
        self.bno.zero()
        self.pid.reset()
        for i in range (10):
            ang_bno = self.bno.readAngleRot()
        
        cm = cm * 100
        self.ser.resetD()
        self.ser.clean()
        
        Fc_avanti = self.fc.read()
        
        rampa = 0
        dis = 0
        
        print("Avanti")
        self.ser.resetD()
        while (dis < cm) and (Fc_avanti[1]) and (Fc_avanti[0]) and (Fc_avanti[2]) and (self.apds.read()[0]>nero) and GPIO.input(17):
            
            ang_bno = self.bno.readAngleRot()
            if ang_bno > 180:
                ang_bno -= 360
#             print(ang_bno)
            dx, sx = self.pid.calcola(ang = ang_bno, offset = centro)
            
            self.ser.writeMot(dx, sx, 1, 1)
#             print (dis)
            if ((self.bno.readAngleInc() <= -15) or (self.bno.readAngleInc() >= 15)):
                print("Rampa")
                if self.bno.readAngleInc() > 0:
                    rampa = 2
                else:
                    rampa = 1
                while ((self.bno.readAngleInc() <= -15) or (self.bno.readAngleInc() >= 15)) and (Fc_avanti[1]):
                    ang_bno = self.bno.readAngleRot()
                    if ang_bno > 180:
                        ang_bno -= 360
                    
                    if rampa == 2:
                        base = 0.8
                    else:
                        base = 1.3
                        
                    dx, sx = self.pid.calcola(ang_bno, base, centro)
                    self.ser.writeMot(base, base, 1, 1)
                    Fc_avanti = self.fc.read()
#                     print(ang_bno)
                    
                self.ser.resetD()
                if rampa == 2:
                    cm = 1300
                else:
                    cm = 1100
                print("Fine Rampa")
            
            time.sleep(0.002)
            self.ser.askD()
            dis = self.ser.read()
            Fc_avanti = self.fc.read()
            if dis > (cm * 0.75):
                centro = finale
        
        self.ser.writeMot()
        return dis, rampa
    
    def indietro(self, cm = 8.0):
        cm = cm * 100
        self.ser.resetD()
        self.ser.writeMot(1.2, 1.2, 2, 2)
        self.ser.clean()
        dis = 0
        print("Indietro")
        self.ser.resetD()
        while dis < cm and GPIO.input(17):
#             self.ser.writeMot(1.2, 1.2, 0, 0) 
            self.ser.askD()
            dis = self.ser.read()
            time.sleep(0.005)
#             print(dis)
        
        self.ser.writeMot()
        self.ser.resetD()
        self.ser.clean()

if __name__ == '__main__':
    mv = MOVIMENTI()
    GPIO.setup(22, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(17, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    while True:
        
        if(GPIO.input(22)):
            while True:
                mv.indietro(300.0)
                time.sleep(2)
