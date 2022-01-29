from vector import *
from aiohttp import web
import socketio
import asyncio
from gpiozero import CPUTemperature
import pygame
import os
import math
import time
from board import SCL, SDA
import busio
import RPi.GPIO as GPIO

# Import the PCA9685 module.
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# constants
SERVOMIN = 604.0 # This is the 'minimum' pulse length count (out of 4096)
SERVOMAX = 2260.0 # This is the 'maximum' pulse length count (out of 4096)
SERVO_FREQ = 50 # Analog servos run at ~50 Hz updates
PCA_OE = 4 # PCA Output Enable Pin

# global variables
powerup = False;
powerdown = False;

# Create the I2C bus interface.
i2c_bus = busio.I2C(SCL, SDA, frequency=400)

# Create a simple PCA9685 class instance.
pca_a = PCA9685(i2c_bus, address=0x40)
pca_b = PCA9685(i2c_bus, address=0x41)

# Set the PWM frequency to 50hz.
pca_a.frequency = SERVO_FREQ
pca_b.frequency = SERVO_FREQ
# Set twice because of some i2c error stuff
time.sleep(0.1);
pca_a.frequency = SERVO_FREQ
pca_b.frequency = SERVO_FREQ

clawOpen = False;
clawClose = False;
clawPos = 0.0;

servos = [None] * 20
servos[0] = servo.Servo(pca_a.channels[8],  actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)
servos[1] = servo.Servo(pca_a.channels[9],  actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)
servos[2] = servo.Servo(pca_a.channels[10], actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)

servos[3] = servo.Servo(pca_a.channels[11],  actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)
servos[4] = servo.Servo(pca_a.channels[12],  actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)
servos[5] = servo.Servo(pca_a.channels[13], actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)

servos[6] = servo.Servo(pca_b.channels[13],  actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)
servos[7] = servo.Servo(pca_b.channels[14],  actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)
servos[8] = servo.Servo(pca_b.channels[15], actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)

servos[9] = servo.Servo(pca_b.channels[2],  actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)
servos[10] = servo.Servo(pca_b.channels[3],  actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)
servos[11] = servo.Servo(pca_b.channels[4], actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)

servos[12] = servo.Servo(pca_b.channels[5],  actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)
servos[13] = servo.Servo(pca_b.channels[6],  actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)
servos[14] = servo.Servo(pca_b.channels[7], actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)

servos[15] = servo.Servo(pca_a.channels[5],  actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)
servos[16] = servo.Servo(pca_a.channels[6],  actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)
servos[17] = servo.Servo(pca_a.channels[7], actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)

servos[18] = servo.Servo(pca_b.channels[0],  actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)
servos[19] = servo.Servo(pca_b.channels[1], actuation_range = 300, min_pulse=SERVOMIN, max_pulse=SERVOMAX)


# Initialize routines
sio = socketio.AsyncServer()
app = web.Application()
sio.attach(app)
loop = asyncio.get_event_loop()
pygame.display.init()

# enable PCA (LOW means On)
GPIO.setmode(GPIO.BCM)
GPIO.setup(PCA_OE, GPIO.OUT)
GPIO.output(PCA_OE, GPIO.LOW)

# setup hexapod parameters
tripodGait = [[0 for x in range(6)] for y in range(120)] 
waveGait = [[0 for x in range(6)] for y in range(120)] 

Hex = gaitEngine()

jX = 0  # (Joystick) Walking X, Y and Rotate
jY = 0
jR = 0

# tripod gait    
Hex.gaitSpeed = 1.0
Hex.legSpeed = 1.0
tripodGait[0][0]  = 1
tripodGait[0][2]  = 1
tripodGait[0][4]  = 1
    
tripodGait[30][1] = 1
tripodGait[30][3] = 1
tripodGait[30][5] = 1
    
tripodGait[60][0] = 1
tripodGait[60][2] = 1
tripodGait[60][4] = 1
    
tripodGait[90][1] = 1
tripodGait[90][3] = 1
tripodGait[90][5] = 1  
    
# wave gait
#Hex.gaitSpeed = 2.0
#Hex.legSpeed = 0.7
waveGait[00][0]  = 1
waveGait[20][4]  = 1
waveGait[40][2]  = 1
waveGait[60][5]  = 1
waveGait[80][1]  = 1
waveGait[100][3] = 1


    
async def controlLoop():
    while True:
        global Hex, powerup, powerdown, clawPos, clawOpen, clawClose, SERVOMIN, SERVOMAX, servos
        IKError = False;
        
        if powerup == True:
            if(Hex.traZ > 0):
                Hex.legS[0].setto(-70, -85, 0)
                Hex.legS[1].setto(-90, 0, 0)
                Hex.legS[2].setto(-70, 85, 0)
                Hex.legS[3].setto(70, 85, 0)
                Hex.legS[4].setto(90, 0, 0)
                Hex.legS[5].setto(70, -85, 0)
                Hex.traZ -= 0.3
            else:     
                powerup = False;               
        elif powerdown == True:
            if(Hex.traZ < 26):
                Hex.legS[0].setto(-65, -70, 0)
                Hex.legS[1].setto(-80, 0, 0)
                Hex.legS[2].setto(-65, 70, 0)
                Hex.legS[3].setto(65, 70, 0)
                Hex.legS[4].setto(80, 0, 0)
                Hex.legS[5].setto(65, -70, 0)
                Hex.traZ += 0.3
            else:
                powerdown = False
        
        clawPos = constrain(clawPos, 0, 67)
        if(clawOpen):
            if(clawPos > 0):
                clawPos -= 2
            else:
                clawOpen = False;
        elif(clawClose):
            if(clawPos < 58):
                clawPos += 2
            else:
                clawClose = False
        
        # Configure and run the hexapod engine
        Hex.gSeq = waveGait
        Hex.walkX = jX
        Hex.walkY = 0.1#test instead of jY
        Hex.walkR = jR
        Hex.gaitStep()
        Hex.runBodyIK()
        IKError = (Hex.runLegIK() != 0)
        
        # servo angle constraints
        angleA = constrain( Hex.legAngle[0][0]*180/PI, -65, 65);
        angleB = constrain(-Hex.legAngle[0][1]*180/PI, -100, 100);
        angleC = constrain( Hex.legAngle[0][2]*180/PI, -100, 100);
        angleD = constrain( Hex.legAngle[1][0]*180/PI, -65, 65);
        angleE = constrain(-Hex.legAngle[1][1]*180/PI, -100, 100);
        angleF = constrain( Hex.legAngle[1][2]*180/PI, -100, 100);
        angleG = constrain( Hex.legAngle[2][0]*180/PI, -65, 65);
        angleH = constrain(-Hex.legAngle[2][1]*180/PI, -100, 100);
        angleI = constrain( Hex.legAngle[2][2]*180/PI, -100, 100);
        
        angleJ = constrain( Hex.legAngle[3][0]*180/PI, -65, 65);  
        angleK = constrain( Hex.legAngle[3][1]*180/PI, -100, 100);
        angleL = constrain(-Hex.legAngle[3][2]*180/PI, -100, 100);
        angleM = constrain( Hex.legAngle[4][0]*180/PI, -65, 65);
        angleN = constrain( Hex.legAngle[4][1]*180/PI, -100, 100);
        angleO = constrain(-Hex.legAngle[4][2]*180/PI, -100, 100);
        angleP = constrain( Hex.legAngle[5][0]*180/PI, -65, 65);
        angleQ = constrain( Hex.legAngle[5][1]*180/PI, -100, 100);
        angleR = constrain(-Hex.legAngle[5][2]*180/PI, -100, 100);
        
        angleS = constrain(Hex.rotZ*180/PI*1.7, -65, 65);
        angleT = constrain(clawPos, 0, 67);
        
        # servo angle offsets
        angleA += -24;
        angleB += 18;
        angleC += 3;
        
        angleD += 14;
        angleE += 2;
        angleF += 0;
        
        angleG += 50;
        angleH += -12;
        angleI += -1;
       
        #---
        
        angleJ += -35;
        angleK += 11;
        angleL += 6;
        
        angleM += 3;
        angleN += 12;
        angleO += -6;
        
        angleP += 23;
        angleQ += 0;
        angleR += 0;
        
        angleS += -8;
        angleT += 0;
        
        
        #print(angleA)
        
        # servo angle mapping
        servos[0].angle = angleA + 90;
        servos[1].angle = angleB + 120;
        servos[2].angle = angleC + 120;
        
        
        servos[3].angle = angleD + 90;
        servos[4].angle = angleE + 120;
        servos[5].angle = angleF + 120;
        
        
        servos[6].angle = angleG + 90;
        servos[7].angle = angleH + 120;
        servos[8].angle = angleI + 120;
        
        servos[9].angle = angleJ + 90;
        servos[10].angle = angleK + 120;
        servos[11].angle = angleL + 120;
        
        servos[12].angle = angleM + 90;
        servos[13].angle = angleN + 120;
        servos[14].angle = angleO + 120;
        
        servos[15].angle = angleP + 90;
        servos[16].angle = angleQ + 120;
        servos[17].angle = angleR + 120;
        
        servos[18].angle = angleS + 120;
        servos[19].angle = angleT + 120;
        
        
        
        
        #pca_a.channels[8].duty_cycle = 0x0000
        #pca_b.channels[0].duty_cycle = 0x0000
        #time.sleep(1)
        #pca_a.channels[8].duty_cycle = 0x7FFF
        #pca_b.channels[0].duty_cycle = 0x3000
        #time.sleep(1)
        
        await asyncio.sleep(0.001) # gait engine loop speed

async def gameLoop():
    global Hex, powerup, powerdown, clawClose, clawOpen, clawPos
    while True:
        try:
            pygame.joystick.init()
            # Check if there is a joystick
            if(pygame.joystick.get_count()) < 1:   
                pygame.joystick.quit()
                await asyncio.sleep(1) 
            else:   
                joystick = pygame.joystick.Joystick(0)
                break
        except pygame.error:
            pygame.joystick.quit()
            await asyncio.sleep(1) 
    while True:       
        try:
            await asyncio.sleep(0.01)
            joystick.init()

            if joystick.get_button(6):
                clawPos += 1
            if joystick.get_button(7):
                clawPos -= 1
            if joystick.get_hat(0)[1] == 1:
                Hex.traX = 0
                Hex.traY = 0
                Hex.traZ = -0.3
            if joystick.get_hat(0)[1] == -1:
                Hex.traX = 0
                Hex.traY = 0
                Hex.traZ = 0.3
                
                
            # Check for events
            for event in pygame.event.get():
                if event.type == pygame.JOYBUTTONDOWN:	
                    if joystick.get_button(3):
                        powerdown = True;
                    elif joystick.get_button(4):
                        powerup = True;
                    elif joystick.get_button(15):
                        Hex.gaitSpeed = 1.0
                        Hex.legSpeed = 1.0
                        Hex.gSeq = list(tripodGait)        
                    elif joystick.get_button(11):
                        Hex.gaitSpeed = 2.0
                        Hex.legSpeed = 1.3
                        Hex.gSeq = list(waveGait)                        
                    elif joystick.get_button(0):
                        clawClose = True
                    elif joystick.get_button(1):
                        clawOpen = True
                        
                if event.type == pygame.JOYAXISMOTION:                        
                    axis0 = joystick.get_axis(0)
                    axis1 = joystick.get_axis(1)
                    axis2 = joystick.get_axis(2)
                    axis3 = joystick.get_axis(3)
                    axis4 = (joystick.get_axis(4) -joystick.get_axis(5))/2

                    jX = axis0*0.7
                    jY = -axis1*1.2
                    jR = axis4*0.8
                    Hex.TraZ = axis2/5
                    Hex.TraY = axis3/4
                    
                # Multiple events are generated for the same axis motion, so break after the first
                break
        except pygame.error:
            await asyncio.sleep(1) 

loop = asyncio.get_event_loop()
loop.create_task(gameLoop())

serialLoop = asyncio.get_event_loop()
serialLoop.create_task(controlLoop())

def index(request):
    with open(os.path.dirname(__file__)+'/index.html') as f:
        return web.Response(text=f.read(), content_type='text/html')

async def sendTemp():
    while True:
        await asyncio.sleep(3)
        await sio.emit('temp', CPUTemperature().temperature)

loop.create_task(sendTemp())

async def heartbeat():
    while True:
        #print('heartbeat')
        await asyncio.sleep(1)

loop.create_task(heartbeat())

@sio.on('connect')
def chat_connection(sid, message):
    print('---- connected ----')
    
    
@sio.on('disconnect')
def chat_disconnect(sid):
    print('---- disconnected ----')

@sio.on('mov')
async def position(sid, msx, msy, msr):
    global jX, jY, jR
    msxString = "{:7.3f}".format(float(msx))
    msyString = "{:7.3f}".format(float(-msy))
    msrString = "{:7.3f}".format(float(msr))
    #print('m %s %s %s'%("000.000",msyString,msrString))
    jX = float(msxString)
    jY = float(msyString)
    jR = float(msrString)
    await asyncio.sleep(0.005)
    
@sio.on('rot')
async def position(sid, yaw, pitch, roll):
    global Hex
    yawString = "{:7.3f}".format(float(yaw/6))
    pitchString = "{:7.3f}".format(float(pitch/6))
    rollString = "{:7.3f}".format(float(roll/6))
    #print('r %s %s %s'%(rollString, pitchString, yawString))
    Hex.rotX = float(rollString)
    Hex.rotY = float(pitchString)
    Hex.rotZ = float(yawString)
    await asyncio.sleep(0.005)

@sio.on('pos')
async def position(sid, message):
    global powerup, powerdown
    if message == 1:
        powerup = True
    if message == 0:
        powerdown = True
    await asyncio.sleep(0.005)
        
@sio.on('gait')
async def position(sid, message):
    if message == 1:
        Hex.gaitSpeed = 1.0
        Hex.legSpeed = 1.0
        Hex.gSeq = list(tripodGait)
    elif message == 0:
        Hex.gaitSpeed = 2.0
        Hex.legSpeed = 1.3
        Hex.gSeq = list(waveGait)
    await asyncio.sleep(0.005)
    
@sio.on('claw')
async def position(sid, message):
    global clawOpen, clawClose
    if message == 1:
        clawClose = True
    elif message == 0:
        clawOpen = True
    await asyncio.sleep(0.005)

@sio.on('power')
async def position(sid, message):
    if message == 1:
        print('power down')

app.router.add_get('/', index)

if __name__ == '__main__':
    web.run_app(app, host='0.0.0.0', port=3000)
    loop.run_forever()
    serialLoop.run_forever()