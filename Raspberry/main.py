import RPi.GPIO as GPIO
import board
import busio
import math
import time
import sys
import queue as Queue

from laser import VL6180
from giroscopio import BNO055
from colore import APDS9960
from led import LED
from seriale import SERIALEPICO
from finecorsa import FINECORSA
from movimenti import MOVIMENTI

from os.path import join

class casella:
    def __init__(self):
        self.padre=0
        self.distanza=10000 

def main(camdx_q, camsx_q, vittima):
    
    GPIO.setmode(GPIO.BCM)
    button_pin = 17
    GPIO.setup(button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    global running
    running = True

    def button_callback(channel):
        global running
        if running==True:
            print("Programma interrotto.")
            running = False
            
            stop=0
#             print(stop)
        else:
            print("Programma ripreso.")
            running = True
            stop=1
    GPIO.setup(27, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    GPIO.setup(4, GPIO.IN, pull_up_down = GPIO.PUD_UP)
    
    GPIO.add_event_detect(button_pin, GPIO.FALLING, callback=button_callback, bouncetime=1000)
        
    
    muro = 150

    
#     mappa=12
# 
#     xStart = xNow = xend = 6
#     yStart = yNow = yend = 6
#     
    xStart = xNow = xend =xck= 40
    yStart = yNow = yend =yck= 40
    
    mappa = 80
    
    ang_attuale = 0
    dire = 1
    calli=0
    ritorno = 0
    
    stop =1
    
    zNow = 1
    
    nero = 410
    blu =550
    grigmin=300
    grigmax=1000
    
    disfc=1500
    ck=1
    chk=0
    sbatto =0
    culo=1
    
    rit=0
    rst=0
    
    laser_mm = [0] * 5
    
    ser = SERIALEPICO()
    ser.writeMot()

    bno = BNO055()
    laser = VL6180()
    apds = APDS9960()
    led = LED()
    fc = FINECORSA()

    mv = MOVIMENTI()
    
#     import subprocess
#     subprocess.check_output(['sudo', 'python3', '-c', "import led; led.lack();" ])
    led.blink(1)
#     led.lack()

    if bno.begin() is not True:
        print('Error initializing BNO')
        exit()

    if apds.begin() is not True:
        print('Error initializing APDS')
        exit()

    board = [['0' for i in range(mappa)] for i in range(mappa)]
    check = [['0' for i in range(mappa)] for i in range(mappa)]
    ctat  = [['0' for i in range(mappa)] for i in range(mappa)]
    #posizione partenza roboto in matrice
    # xStart = 
    # yStart = 
    # 
    # xNow = 
    # yNow = 
    # 
    # xarrivo = 
    # yarrivo = 
    # variabili test


    #se vuoto valore 0
    #se muro valore !


    board[yStart][xStart] = '1'
    #cambiare in base a quando grande e' la mappa x2
    for i in range(mappa):
        for j in range(mappa):
            if i%2 != 0:
                board[i][j] = '-'
            if j%2 != 0:
                board[i][j] = '-'
            if j%2 != 0 and i%2 != 0:
                board[i][j] = '$'

    GPIO.setup(26,GPIO.OUT)
    GPIO.output(26,GPIO.HIGH)

    robot = board[yNow][xNow]
    
#     board[38][56]='4'
#     board[36][56]='4'
#     
                 
#     start_time = time.time()
    while True:
        
        
        if running:
            
#             start_time = time.time()       
            rst=0
        
            ang_attuale = ang_attuale % 360
            
            x=int(math.cos(math.radians(ang_attuale)))
            y=int(math.sin(math.radians(ang_attuale)))
            
            for i in range(5):
                lett = laser.read(i)
                laser_mm[i] = lett
            
            bno.zero()
            
             #RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO
            if ck==1:    
                if ((laser_mm[1] < muro) and (laser_mm[2] < muro)): #RADDRIZZO DX
                    ang_DX = math.degrees(math.atan((laser_mm[1] - laser_mm[2])/175))
                    
                    if (ang_DX < -8.0) or (ang_DX > 8.0):
                        print('raddrizzo DX', ang_DX)
#                         bno.begin()
                        bno_ANG = bno.readAngleRot()
                        if (ang_DX > 0):
                            ser.writeMot(0.3, 0.3, 1, 2)
                            ang_DX = ang_DX - 3
                            while (bno_ANG < ang_DX) or (bno_ANG > 350) and GPIO.input(17):
                                bno_ANG = bno.readAngleRot()
                        
                        else:
                            ang_DX = ang_DX + 363
                            ser.writeMot(0.3, 0.3, 2, 1)
                            while (bno_ANG < 10) or (bno_ANG > ang_DX) and GPIO.input(17):
                                bno_ANG = bno.readAngleRot()
                    
                    ser.writeMot()
                
                elif ((laser_mm[3] < muro) and (laser_mm[4] < muro)): #RADDRIZZO SX
                    ang_SX = math.degrees(math.atan((laser_mm[4] - laser_mm[3])/175))
                    
                    if (ang_SX < -8.0) or (ang_SX > 8.0):
                        print('raddrizzo SX', ang_SX)
#                         bno.begin()
                        bno_ANG = bno.readAngleRot()
                        if (ang_SX > 0):
                            ang_SX = ang_SX - 3
                            ser.writeMot(0.3, 0.3, 2, 1)
                            while (bno_ANG < ang_SX) or (bno_ANG > 350) and GPIO.input(17):
                                bno_ANG = bno.readAngleRot()
                        
                        else:
                            ang_SX = ang_SX + 363
                            ser.writeMot(0.3, 0.3, 1, 2)
                            while (bno_ANG < 10) or (bno_ANG > ang_SX) and GPIO.input(17):
                                bno_ANG = bno.readAngleRot()
                    
                    ser.writeMot()
            ck=1
            
#             GPIO.output(26,GPIO.LOW)
#             time.sleep(0.1)
#             GPIO.output(26,GPIO.HIGH)
            #FINE FINE RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO#RADDRIZZO   
            
            check_avanti = 0
            if ritorno==0:
                if ((laser.read(1) < muro) and (laser.read(2) < muro)):
                    if board[yStart][xStart] != '2':
                        print()
                        try:
                            camdx_q.put_nowait(True)
                        except Queue.Full:
                            None
                    board[yNow + y][xNow + x] = '!'

                if ((laser.read(3) < muro) and (laser.read(4) < muro)):
                    if board[yStart][xStart] != '2':
                        print()
                        try:
                            camsx_q.put_nowait(True)
                        except Queue.Full:
                            None
                    board[yNow - y][xNow - x] = '!'
                
                if laser.read(0) < muro:
                    board[yNow - x][xNow + y] = '?'
                    if board[yStart][xStart] != '2':
                        check_avanti = 1
                
                time.sleep(0.2)
                try:
                    vit = vittima.get_nowait()
                except Queue.Empty:
                    vit = -1
                    pass
                if vit>=0:
                    led.blink(5)
            board[yStart][xStart] = '1' 
# mappa   ## mappa  # mappa   mappa  # mappa  # mappa   ## mappa  # mappa   mappa  # mappa # mappa   ## mappa  # mappa   mappa  # mappa # mappa   ## mappa  # mappa   mappa  # mappa          

#         
#             print('angolo')
#             print(ang_attuale) 
#             for i in range(mappa):
#                 for j in range(mappa):
#                     print(' ',board[i][j], end = '')
#                  
#                 print()
#                 print() 
    #         
            
            #SCELGO direzione#SCELGO direzione#SCELGO direzione#SCELGO direzione#SCELGO direzione#SCELGO direzione#SCELGO direzione
            
            led.ok()
            
            #     scelgo via
        
            if (board[yNow + y][xNow + x] != '!') and (board[yNow + y][xNow + x] != '?') and (board[yNow + (y*2)][xNow + (x*2)] != '4'):
                if (board [yNow + (y*2)][xNow + (x*2)] == '2') and (board[yNow - x][xNow + y] != '!') and (board[yNow - x][xNow + y] != '?') and (board[yNow - (x*2)][xNow + (y*2)] != '4') and (board[yNow - (x*2)][xNow + (y*2)] != '2'):
                    yNow -= (x*2)
                    xNow += (y*2)
                    direzione = 1
                    ang_attuale += 0
                elif (board [yNow + (y*2)][xNow + (x*2)] == '2') and (board[yNow - y][xNow - x] != '!') and (board[yNow - y][xNow - x] != '?') and (board[yNow - (y*2)][xNow - (x*2)] != '4') and (board[yNow - (y*2)][xNow - (x*2)] != '2'):
                    xNow -= (x*2)
                    yNow -= (y*2)   
                    direzione = 4
                    ang_attuale += 270 
                elif(board[yNow + (y*2)][xNow + (x*2)] != '2'):
                    xNow += (x*2)
                    yNow += (y*2)    
                    direzione = 2
                    ang_attuale += 90
                    
                else:
                    calli=1
            
            elif (board[yNow - x][xNow + y] != '!') and (board[yNow - x][xNow + y] != '?') and (board[yNow - (x*2)][xNow + (y*2)] != '4'):
                if (board[yNow - (x*2)][xNow + (y*2)] == '2') and (board[yNow - y][xNow - x] != '!') and (board[yNow - y][xNow - x] != '?') and (board[yNow - (y*2)][xNow - (x*2)] != '4') and (board[yNow - (y*2)][xNow - (x*2)] != '2'):
                    xNow -= (x*2)
                    yNow -= (y*2)            
                    direzione = 4
                    ang_attuale += 270
                elif(board[yNow - (x*2)][xNow + (y*2)] != '2'):
                    yNow -= (x*2)
                    xNow += (y*2)
                    direzione = 1
                    ang_attuale += 0
                else:
                    calli=1
            
            elif (board[yNow - y][xNow - x] != '!') and (board[yNow - y][xNow - x] != '?') and (board[yNow - (y*2)][xNow - (x*2)] != '4'):
                
                if (board[yNow - (y*2)][xNow - (x*2)] == '2'):
                    calli=1
                else:     
                    xNow -= (x*2)
                    yNow -= (y*2)
                    direzione = 4
                    ang_attuale += 270
            
           
            else:
                
                calli=1
            
            ang_attuale = ang_attuale % 360
            print(ang_attuale)
            if calli==1:
                print("faccio calli")
                yn=yNow//2
                xn=xNow//2
                dis=0
                
                arrivo=[]
                mpcalli = [[casella() for i in range(mappa//2)] for i in range(mappa//2)]
                frontiera= [xn,yn,dis]
                mpcalli[yn][xn].distanza = 0
                while len(arrivo) == 0:
#                     print (frontiera)
                    if len (frontiera) == 0 and ritorno == 0 and not(yend==yNow and xend == xNow):
                        print ('ritorno')
                        ritorno = 1
                        board[yend][xend]= '0'
                        yn=yNow//2
                        xn=xNow//2
                        dis=0
                        arrivo=[]
                        mpcalli = [[casella() for i in range(mappa//2)] for i in range(mappa//2)]
                        frontiera= [xn,yn,dis]
                        mpcalli[yn][xn].distanza = 0
                    if len (frontiera) ==0 and (ritorno == 1 or (yend==yNow and xend == xNow)):
                        
                        print('finito')
                        GPIO.cleanup()
                        fine = 1
                        return fine
#                     print (ritorno)
                    xn=frontiera[0]
                    yn=frontiera[1]
                    dis=frontiera[2]
                    dis +=1
                    if (board[(yn*2)][(xn*2) + 1]!= '!') and (board[yn*2][(xn*2) + 1]!= '?') and (board[yn*2][(xn*2) + 2]!= '4'):
                        
                        if board[yn*2][(xn*2)+2]=='2' and mpcalli[yn][xn].distanza<mpcalli[yn][xn+1].distanza:
                            mpcalli[yn][xn+1].distanza = dis
                            mpcalli[yn][xn+1].padre = 2
                            frontiera.append(xn+1)
                            frontiera.append(yn)
                            frontiera.append(dis)

                        elif board[yn*2][(xn*2) + 2]== '0':
                            arrivo.append(xn+1)
                            arrivo.append(yn)
                            arrivo.append(dis)
                            mpcalli[yn][xn+1].distanza = dis
                            mpcalli[yn][xn+1].padre = 2


                    
                    if (board[(yn*2)-1][(xn*2)]!= '!') and (board[(yn*2)-1][(xn*2)]!= '?') and (board[(yn*2)-2][(xn*2)]!= '4'):
                        if board[(yn*2)-2][(xn*2)]=='2' and mpcalli[yn][xn].distanza<mpcalli[yn-1][xn].distanza:
                            mpcalli[yn-1][xn].distanza = dis
                            mpcalli[yn-1][xn].padre = 1
                            frontiera.append(xn)
                            frontiera.append(yn-1)
                            frontiera.append(dis)

                        elif board[(yn*2)-2][(xn*2)]== '0':
                            arrivo.append(xn)
                            arrivo.append(yn-1)
                            arrivo.append(dis)
                            mpcalli[yn-1][xn].distanza = dis
                            mpcalli[yn-1][xn].padre = 1
                    
                    if (board[(yn*2)][(xn*2) - 1]!= '!') and (board[yn*2][(xn*2) - 1]!= '?') and (board[yn*2][(xn*2) - 2]!= '4'):
                        if board[yn*2][(xn*2)-2]=='2' and mpcalli[yn][xn].distanza<mpcalli[yn][xn-1].distanza:
                            mpcalli[yn][xn-1].distanza = dis
                            mpcalli[yn][xn-1].padre = 4
                            frontiera.append(xn-1)
                            frontiera.append(yn)
                            frontiera.append(dis)

                        elif board[yn*2][(xn*2) - 2]== '0':
                            arrivo.append(xn-1)
                            arrivo.append(yn)
                            arrivo.append(dis)
                            mpcalli[yn][xn-1].distanza = dis
                            mpcalli[yn][xn-1].padre = 4
                    
                    if (board[(yn*2)+1][(xn*2)]!= '!') and (board[(yn*2)+1][(xn*2)]!= '?') and (board[(yn*2)+2][(xn*2)]!= '4'):
                        if board[(yn*2)+2][(xn*2)]=='2' and mpcalli[yn][xn].distanza<mpcalli[yn+1][xn].distanza:
                            mpcalli[yn+1][xn].distanza = dis
                            mpcalli[yn+1][xn].padre = 3
                            frontiera.append(xn)
                            frontiera.append(yn+1)
                            frontiera.append(dis)

                        elif board[(yn*2)+2][(xn*2)]== '0':
                            arrivo.append(xn)
                            arrivo.append(yn+1)
                            arrivo.append(dis)
                            mpcalli[yn+1][xn].distanza = dis
                            mpcalli[yn+1][xn].padre = 3
                    
                    
#                     for i in range(mappa // 2):
#                         for j in range(mappa // 2):
#                             print(' ',mpcalli[i][j].padre, end = '')
#                          
#                         print()
#                         print()
#                     print()
#                         
                        
#                     print(frontiera)
#                     time.sleep(10)
                        
                    for i in range (3):
                        frontiera.pop(0)
                
                frontiera.clear()
                xn=arrivo[0]
                yn=arrivo[1]
                dis=arrivo[2]
                while dis!=1:
                    dis -=1
                    if mpcalli[yn][xn].padre==4:
                        xn+=1
                    elif mpcalli[yn][xn].padre==2:
                        xn-=1
                    elif mpcalli[yn][xn].padre==3:
                        yn-=1
                    elif mpcalli[yn][xn].padre==1:
                        yn+=1
#                 for i in range(mappa // 2):
#                         for j in range(mappa // 2):
#                             mpcalli[i][j].padre=0
#                 
    #             print (((ang_attuale) % 360))
    #             print ((mpcalli[yn][xn].padre-1)*90)

                if ((ang_attuale % 360))==(mpcalli[yn][xn].padre-1)*90:
                    yNow -= (x*2)
                    xNow += (y*2)
                    direzione = 1
                    ang_attuale += 0
                    
                elif ((ang_attuale+90) % 360)==(mpcalli[yn][xn].padre-1)*90:
                    xNow += (x*2)
                    yNow += (y*2)    
                    direzione = 2
                    ang_attuale += 90
                    
                elif ((ang_attuale+180) % 360)==(mpcalli[yn][xn].padre-1)*90 :
                    yNow += (x*2)
                    xNow -= (y*2)
                    direzione = 3
                    ang_attuale += 180
                    
                elif((ang_attuale+270) % 360)==(mpcalli[yn][xn].padre-1)*90 :
                    xNow -= (x*2)
                    yNow -= (y*2)
                    direzione = 4
                    ang_attuale += 270
                arrivo.clear()
            
            #FINE FINE SCELGO direzione#SCELGO direzione#SCELGO direzione#SCELGO direzione#SCELGO direzione#SCELGO direzione#SCELGO direzione
            ang_attuale = ang_attuale % 360
            
            x=int(math.cos(math.radians(ang_attuale)))
            y=int(math.sin(math.radians(ang_attuale)))
            
            vit = -1
            
            bno_ANG = bno.readAngleRot()
            
            if bno_ANG > 180:
                bno_ANG -= 360
            bno_offset = bno_ANG
            if direzione == 1:
                print('Vado dritto')
            else:
                bno_ANG = bno.readAngleRot()
                if bno_ANG > 180:
                    bno_ANG -= 360
                bno_ANG -= bno_offset
                if direzione == 2:
                    print('Ruoto di 90 gradi a destra')
                    ser.writeMot(1.5, 1.5, 1, 2)
                    ser.writeMot(1.5, 1.5, 1, 2)
                    while (bno_ANG < 77) and GPIO.input(17):
#                         ser.writeMot(1.0, 1.0, 1, 2)
                        bno_ANG = bno.readAngleRot()
                        if bno_ANG > 180:
                            bno_ANG -= 360
                        bno_ANG -= bno_offset
                        #print(bno_ANG)
#                         print(stop)
                    ser.writeMot()
                   
                    if check_avanti == 1:
                        try:
                            camsx_q.put_nowait(True)
                        except Queue.Full:
                            None
                    
                elif direzione == 4:
                    print('Ruoto di 90 gradi a sinistra')
                    ser.writeMot(1.5, 1.5, 2, 1)
                    ser.writeMot(1.5, 1.5, 2, 1)
                    while (bno_ANG > -77) and GPIO.input(17):
#                         ser.writeMot(1.0, 1.0, 2, 1)
                        bno_ANG = bno.readAngleRot()
                        if bno_ANG > 180:
                            bno_ANG -= 360
                        bno_ANG -= bno_offset
                        #print(bno_ANG)
                    ser.writeMot()
                   
                    if check_avanti == 1:
                        try:
                            camdx_q.put_nowait(True)
                        except Queue.Full:
                            None
                   
                elif direzione == 3:
                    print('Ruoto di 180 gradi a destra')
                    bno.zero()
                    bno_ANG = bno.readAngleRot()
                    if bno_ANG > 180:
                        bno_ANG -= 360
                    bno_offset = 0
                    if check_avanti == 1:
                        ser.writeMot(1.0, 1.0, 1, 2)
                        ser.writeMot(1.0, 1.0, 1, 2)
                        while (bno_ANG < 82.5) and GPIO.input(17):
#                             ser.writeMot(1.0, 1.0, 1, 2)
                            bno_ANG = bno.readAngleRot()
                            if bno_ANG > 180:
                                bno_ANG -= 360
                            bno_ANG -= bno_offset
                            #print(bno_ANG)
                        ser.writeMot()
                        try:
                            camsx_q.put_nowait(True)
                        except Queue.Full:
                            None
                        time.sleep(0.2)   
                        try:
                            vit = vittima.get_nowait()
                        except Queue.Empty:
                            vit = -1
                            pass
                        if vit>=0:
                            led.blink(5)
                        check_avanti = 0
                        bno_ANG = bno.readAngleRot()
                        ser.writeMot(1.0, 1.0, 1, 2)
                        ser.writeMot(1.0, 1.0, 1, 2)
                        while (bno_ANG < 170) and GPIO.input(17):
#                             ser.writeMot(1.0, 1.0, 1, 2)
                            bno_ANG = bno.readAngleRot()
                            if bno_ANG > 180:
                                bno_ANG -= 360
                            bno_ANG -= bno_offset
                            #print(bno_ANG)
                        ser.writeMot()
                        
                    else:
                        ser.writeMot(0.7, 0.7, 1, 2)
                        ser.writeMot(0.7, 0.7, 1, 2)
                        while (bno_ANG < 170) and GPIO.input(17):
                            bno_ANG = bno.readAngleRot()
                            if bno_ANG > 180:
                                bno_ANG -= 360
                            bno_ANG -= bno_offset
                            #print(bno_ANG)
                        ser.writeMot()
            direzione=0            
            if check_avanti == 1:
                time.sleep(0.2)
            
#             print ('cosa vedo dietro')
#             print (board[yNow + x][xNow - y])
#             print(ang_attuale)
#             print(x)
#             print (y)
#             print(xNow)
#             print(yNow)
            try:
                vit = vittima.get_nowait()
            except Queue.Empty:
                vit = -1
                pass
            if vit>=0:
                led.blink(5)
            
            
            if ((board[yStart + x][xStart - y] == '!') or (board[yStart + x][xStart - y] == '?')) and culo==1:
                mv.indietro()
#                 mv.avanti(cm = 3)
                dist_avanti = 215
                print('sbatto')
                sbatto=1
                ck=0
            else:
                dist_avanti = 190
            culo=1
            
            board[yStart][xStart] = '2'
            calli=0
                
            check_avanti = 0
            
            rampa = 0
            
            for i in range(5):
                lett = laser.read(i)
                laser_mm[i] = lett
            
            if laser_mm[1] < muro and laser_mm[2] < muro:
                
                attuale = math.degrees(math.atan((laser_mm[1] - laser_mm[2])/175))
                centro = math.degrees(math.atan((70 - (laser_mm[1] + laser_mm[2])/2)/dist_avanti))
                print("CENTRO DX")
                
            elif laser_mm[3] < muro and laser_mm[4] < muro:
                
                attuale = math.degrees(math.atan((laser_mm[3] - laser_mm[4])/175))
                centro = math.degrees(math.atan((((laser_mm[4] + laser_mm[3])/2)-70)/dist_avanti))
                print("CENTRO SX")
                
            else:
                attuale = 0
                centro = 0
            
            voluto = (attuale - centro) / 2
            
#             print("Attuale: ",attuale)            
#             print("Centro: ",centro)
#             print("Voluto: ",voluto)
                
            #AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI
            if sbatto==1:
                dis, rampa = mv.avanti(cm = 29.5, nero = nero, centro = voluto, finale = attuale)
            else:
                dis, rampa = mv.avanti(nero = nero, centro = voluto, finale = attuale)
            sbatto = 0
                        
            if rampa == 1 and zNow == 0:
                #SALGO
                print("salgo")
                zNow += 1
            elif rampa == 2 and zNow == 1:
                #SCENDO
                print("Scendo")
                rampa = 1
                zNow -= 1
            else:
                rampa = 0
            
            if rampa==1:
                board[yNow][xNow]='2'
                board[yNow+y][xNow+x]='!'
                board[yNow-y][xNow-x]='!'
                yNow -= 2*x
                xNow += 2*y
#             print (dis)
#             print ('distanza gg')
            #AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI#AVANTI
            
            if (apds.read()[0]<nero) and (apds.read()[3]<blu):
                print('Negro')
                mv.indietro(dis/100)
                
                board[yNow][xNow] = '4'
                yNow = yStart
                xNow = xStart
                
#                 ang_attuale += 180
#                 
#                 bno.begin()
#                 print('Ruoto di 180 gradi')
#                 bno_ANG = bno.readAngleRot()
#                 ser.writeMot(0.7, 0.7, 1, 2)
#                 while (bno_ANG < 173) or (bno_ANG > 190) and GPIO.input(17):
# #                     ser.writeMot(0.7, 0.7, 1, 2)
#                     bno_ANG = bno.readAngleRot()
#                    # print(bno_ANG)
#                 ser.writeMot()
                
            
            elif(apds.read()[1]<grigmax) and (apds.read()[1]>grigmin):
                led.lack()
                print('check')
                check.clear()
                check = board.copy()
                xck= xNow
                yck = yNow
                chk=1
            elif(apds.read()[3]<blu):
                print('Blu')
                time.sleep(4.5)
                
            ctat = board.copy()
                
            Fc_avanti = fc.read()
            if not(Fc_avanti[1]):
                if dis<1000:
                    print('finecorsa -1 c')
                    mv.indietro(dis/100)
                    xNow=xStart
                    yNow=yStart
                else:
                    print('finecorsa c')
                    mv.indietro(3)
            if not(Fc_avanti[2]) and not (Fc_avanti[0]):
                 if dis<disfc:
                    print('finecorsa -1 e')
                    mv.indietro(dis/100)
                    xNow=xStart
                    yNow=yStart
                 else:
                    print('finecorsa e')
                    mv.indietro(3) 
            
            elif not(Fc_avanti[0]) and (Fc_avanti[2]):
                if dis<disfc:
                    print('finecorsa -1 s')
                    xNow=xStart
                    yNow=yStart
                bno.zero()
                print('slida destra s')#finecorsa sinistra
                ser.writeMot(2.0, 0.5, 2, 2)
                time.sleep(0.5)
                bno_ANG = bno.readAngleRot()
                culo=0
                while (bno_ANG >300) or (bno_ANG <1) and GPIO.input(17):
                    ser.writeMot(0.7, 0.7, 1, 2)
                    bno_ANG = bno.readAngleRot()
                   # print(bno_ANG)
                ser.writeMot()
                ck=0
                
                                
            elif not(Fc_avanti[2]) and (Fc_avanti[0]):
                if dis<1000:
                    print('finecorsa -1 d')
                    xNow=xStart
                    yNow=yStart
                bno.zero()
                print ('slide sinistra d')#finecorsa destra
                ser.writeMot(0.5, 2.0, 2, 2)
                time.sleep(0.5)
                bno_ANG = bno.readAngleRot()
                culo =0
                while (bno_ANG >359) or (bno_ANG <30)  and GPIO.input(17):
                    ser.writeMot(0.7, 0.7, 2, 1)
                    bno_ANG = bno.readAngleRot()
                   # print(bno_ANG)
                ser.writeMot()
                ck=0
           
            
            if yStart != yNow or xStart != xNow :
                yStart = yNow
                xStart = xNow
        #         board[yNow][xNow] = '1'
            
#             print()
#             print(ang_attuale)
            
#             end_time = time.time()
#             delta = end_time - start_time
#             print(delta)
#             print(xNow)
        else:
            # Il programma è in pausa
            print("Programma in pausa...")
            
            if rst == 0:
                led.lack()
                board.clear()
                board = check.copy()
                xNow=xStart=xck
                ang_attuale=0
                yNow=yStart=yck
                time.sleep(0.1)
                print('mappa ck')
            if (not GPIO.input(4))or rst==2:
                led.ok()
                rst=2
                print('ctatttc')    
#                 board = ctat.copy()
                xNow=xStart=xck
                ang_attuale=0
                yNow=yStart=yck
                time.sleep(0.1)
                led.stop()
                time.sleep(0.1)
                if rit==0:
                    for i in range(mappa):
                        for j in range(mappa):
                            if board[i][j]=='0':
                                board[i][j]='4'
                    print ('ritorno')
#                     ritorno = 1
#                     board[yend][xend]= '0'
#                     yn=yNow//2
#                     xn=xNow//2
#                     dis=0
#                     arrivo=[]
#                     mpcalli = [[casella() for i in range(mappa//2)] for i in range(mappa//2)]
#                     frontiera= [xn,yn,dis]
#                     mpcalli[yn][xn].distanza = 0
                    rit=1
#                     ritorno =1
                    board[yend][xend]= '0'
                    
        
            if (not GPIO.input(27)) or rst==1:
                led.stop()
                print('resettt')
                rst=1
                ctat.clear()
                board.clear()
                check.clear()
                board = [['0' for i in range(mappa)] for i in range(mappa)]
                check = [['0' for i in range(mappa)] for i in range(mappa)]
                board[yStart][xStart] = '1'
                #cambiare in base a quando grande e' la mappa x2
                for i in range(mappa):
                    for j in range(mappa):
                        if i%2 != 0:
                            board[i][j] = '-'
                        if j%2 != 0:
                            board[i][j] = '-'
                        if j%2 != 0 and i%2 != 0:
                            board[i][j] = '$'
                xStart = xNow = xend =xck= 40
                yStart = yNow = yend =yck= 40
                chk=0
                ck=1
                ritorno =0
                
                
        
            

    # base bianca=2 /base blu=3 /base nera =4 
