'''
Hexapod Simulator
 2021 Maximilian Kern
'''

from vector import *
import zipfile

tripodGait = [[0 for x in range(6)] for y in range(120)] 
waveGait = [[0 for x in range(6)] for y in range(120)] 

Hex = gaitEngine()

camRotX = float()  # Camera rotation and translation
camRotY = float()

jX = 0  # (Joystick) Walking X, Y and Rotate
jY = 0
jR = 0

gX = 0  # Ground X, Y, and Rotation
gY = 0
gR = 0

showUI = True
img = PGraphics()

bodyObj = PShape()
coxaObj = PShape()
femurObj = PShape()
tibiaObj = PShape()
coxaMObj = PShape()
femurMObj = PShape()
tibiaMObj = PShape()


def setup():

    size(700, 700, P3D)  # , OPENGL)
    rectMode(CENTER)

    frameRate(50)
    
    global Hex

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
    

    global bodyObj, coxaObj, coxaMObj, femurObj, tibiaObj, tibiaMObj
    
    unzip(sketchPath("data.zip"))

    bodyObj = loadShape("Body.obj")
    coxaObj = loadShape("Coxa.obj")
    coxaMObj = loadShape("CoxaM.obj")
    femurObj = loadShape("Femur.obj")
    tibiaObj = loadShape("Tibia.obj")
    tibiaMObj = loadShape("TibiaM.obj")
    
    tibiaObj.scale(1000, 1000, 1000)
    tibiaMObj.scale(1000, 1000, 1000)
    coxaObj.scale(1000, 1000, 1000)
    coxaMObj.scale(1000, 1000, 1000)
    bodyObj.scale(1000, 1000, 1000)
    femurObj.scale(1000, 1000, 1000)
    

    global img
    img = createGraphics(500, 500)
    img.beginDraw()
    for i in range(0, width):
        for j in range(0, height):
            img.set(i, j, color(random(100, 200)))

    img.endDraw()

    stroke(255)
    
    

def draw():
    global showUI
    global camRotX, camRotY
    global jX, jY, jR
    global gX, gY, gR
    global img
    global Hex

    translate(+width / 2, +height / 2)
    scale(2)
    translate(-width / 2, -height / 2)
    smooth(8)
    fill(0)
    background(255)
    lights()
    directionalLight(51, 102, 126, -1, 0, 0)
    directionalLight(51, 102, 126, 1, 0, 0)
    # Camera Rotation
    translate(width / 2, height / 2)

    rotateX(-camRotY)
    rotateZ(camRotX)

    # Calculate rotation and translation of ground below robot
    sinG = sin(gR / 100)
    cosG = cos(gR / 100)
    gX -= jX * cosG - jY * sinG
    gY += jY * cosG + jX * sinG
    gR -= jR

    if(showUI and camRotY > -PI / 2):
        pushMatrix()
        rotate(gR / 100)
        translate(gX - 10 * img.width / 2, gY - 10 * img.height / 2, -0.01)
        scale(10)
        image(img, 0, 0)
        popMatrix()
        fill(255)
        #text("Left Mouse Drag or [W][A][S][D] to move", 10, 20)
        #text("Right Mouse Drag or [Q][E] to rotate", 10, 40)
        #text("Middle Mouse Drag to look around", 10, 60)
        #text("jX " + nf(jX, 0, 2) + " jY " + nf(jY, 0, 2) + " jR " + nf(jR, 0, 2), 10, 80)
        for i in range(0, 6):
            pushMatrix()  # Verify IK by using target coordinates directly
            fill(255)
            text(i, Hex.leg[i].x+10, Hex.leg[i].y+10)
            noFill()
            rect(Hex.legT[i].x, Hex.legT[i].y, 20, 20)
            rect(Hex.legS[i].x, Hex.legS[i].y, 20+Hex.targetDeadZone*2, 20+Hex.targetDeadZone*2)
            translate(Hex.leg[i].x, Hex.leg[i].y, Hex.leg[i].z)
            sphere(3)
            popMatrix()

    # compensate for body translation/rotation
    rotateX(Hex.rotY)
    rotateY(Hex.rotX)
    rotateZ(Hex.rotZ)
    translate(-Hex.traX, -Hex.traY, -Hex.traZ)

    # Configure and run the hexapod engine
    Hex.gSeq = waveGait
    Hex.walkX = jX
    Hex.walkY = jY
    Hex.walkR = jR
    Hex.gaitStep()
    Hex.runBodyIK()
    Hex.runLegIK()
    

    if(showUI):
        # Draw body polygon
        beginShape()
        noFill()
        for i in range(0, 6):
           vertex(Hex.coxaPos[i].x, Hex.coxaPos[i].y, Hex.coxaPos[i].z)

        endShape(CLOSE)

        # Draw leg support polygon
        beginShape()
        if(camRotY > -PI / 2):
          fill(98, 187, 227, 100)
        for i in range(0, 6):
          if (Hex.legIK[i].z == 0): vertex(Hex.legIK[i].x, Hex.legIK[i].y)

        endShape(CLOSE)

    pushMatrix();
    translate(0, 0, Hex.coxaPos[0].z + 26);
    shape(bodyObj);
    popMatrix();
  
    # Draw Legs
    for i in range(0, 6):
      pushMatrix()
      translate(Hex.coxaPos[i].x, Hex.coxaPos[i].y, Hex.coxaPos[i].z)
      rotateZ(Hex.legAngle[i][0])
      if (i>2): rotateZ(PI)
      pushMatrix()
      rotateZ(-PI/2)
      translate(0, 0, -3)
      if (i<3):
        shape(coxaObj)
      else:
        rotateY(PI)
        shape(coxaMObj)
    
      popMatrix()
      if(showUI): line(0, 0, 0, -Hex.coxa, 0, 0)

      translate(-Hex.coxa, 0, 0)
      rotateY(PI-Hex.legAngle[i][1])
      pushMatrix()
      rotateZ(-PI/2)
      rotateX(PI)
      if (i<3):
        translate(14, 0, 0)
        shape(femurObj)
      else:
        rotateY(PI)
        rotateX(PI)
        translate(14, 44, 0)
        shape(femurObj)
    
      popMatrix()
      if(showUI): line(0, 0, 0, Hex.femur, 0, 0)

      translate(Hex.femur, 0, 0)
      rotateY(-PI/2-Hex.legAngle[i][2])
      pushMatrix()
      if (i<3):
        translate(0, 15, 0)
        rotateY(PI/2)
        rotateZ(PI)
        shape(tibiaMObj)
      else:
        translate(0, -15, 0)
        rotateY(-PI/2)
        shape(tibiaObj)
    
      popMatrix()
      if(showUI): line(0, 0, 0, Hex.tibia, 0, 0)   
      popMatrix()
      
      

def mouseReleased():
    global jX, jY, jR
    jX = 0
    jY = 0
    jR = 0

def mouseDragged():
    global jX, jY, jR, camRotX, camRotY

    if (mouseButton == LEFT):
        jX += (mouseX - pmouseX) * 0.01
        jY -= (mouseY - pmouseY) * 0.01

    if (mouseButton == RIGHT):
        jR += (mouseX - pmouseX) * 0.01

    if (mouseButton == CENTER):
        camRotX -= (mouseX - pmouseX) * 0.01
        camRotY += (mouseY - pmouseY) * 0.01

def keyPressed():
    global jX, jY, jR, showUI
    if (key == 'a'): jX = -0.5
    if (key == 'd'): jX = 0.5
    if (key == 's'): jY = -0.5
    if (key == 'w'): jY = 0.5
    if (key == 'q'): jR = -0.5
    if (key == 'e'): jR = 0.5
    if (key == 'u'): showUI = not showUI

def keyReleased():
    global jX, jY, jR
    if (key == 'd' or key == 'a'): jX = 0
    if (key == 's' or key == 'w'): jY = 0
    if (key == 'e' or key == 'q'): jR = 0
    
def unzip(path_to_zip_file):
    with zipfile.ZipFile(path_to_zip_file, 'r') as zip_ref:
        zip_ref.extractall("data")
