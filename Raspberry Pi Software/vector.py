import math

def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

def mapp(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def sq(val):
    return (val*val)

def sin(val):
    return math.sin(val)

def cos(val):
    return math.cos(val)

def sqrt(val):
    return math.sqrt(val)

def atan(val):
    return math.atan(val)

def acos(val):
    return math.acos(constrain(val, -1, 1))

PI = math.pi



class gaitEngine(object):

    # public:
    gaitStepN = 0
    gSeq = [[0 for x in range(6)] for y in range(120)]
    legMoving = [0]*6
    gaitSpeed = 1.0 #  control gait speed by adjusting gaitStepN progression
    legSpeed = 1.0 #  control gait speed by adjusting leg lift function
    stepHeight = 20
    targetDeadZone = 3 #  allow off center leg position
    walkX = 0 #  inputs for walking in X,Y and rotation
    walkY = 0
    walkR = 0
    traX = float() # current body translation
    traY = float()
    traZ = float()
    rotX = float() # current for body rotation
    rotY = float()
    rotZ = float()
    #  hexapod constants
    coxa = int(11.6)
    femur = 44
    tibia = 69
    
    coxaPos = [] #  body position
    leg = [] # current leg positions (without body translation/rotation)
    legS = [] # defaulegT leg positions
    legT = [] # target leg positions
    legIK = [] # leg positions for body translation/rotation
    
    legAngle = [[0 for x in range(3)] for y in range(6)]

    def __init__(self): # Initialization
        
        for i in range(0, 6):
            self.coxaPos.append(vector())
            self.leg.append(vector())
            self.legS.append(vector())
            self.legT.append(vector())
            self.legIK.append(vector())
            self.coxaPos.append(vector())
            
        self.leg[0].setto(-70, -85, 0)
        self.leg[1].setto(-90, 0, 0)
        self.leg[2].setto(-70, 85, 0)
        self.leg[3].setto(70, 85, 0)
        self.leg[4].setto(90, 0, 0)
        self.leg[5].setto(70, -85, 0)
        
        for i in range(0, 6):
            self.legS[i]._setto(self.leg[i])
            self.legT[i]._setto(self.leg[i])
                
        bodyHeight = 40
        self.coxaPos[0].setto(-28, -48, bodyHeight)
        self.coxaPos[1].setto(-35, 0, bodyHeight)
        self.coxaPos[2].setto(-28, 48, bodyHeight)
        self.coxaPos[3].setto(28, 48, bodyHeight)
        self.coxaPos[4].setto(35, 0, bodyHeight)
        self.coxaPos[5].setto(28, -48, bodyHeight)

    def gaitStep(self):
        
        self.walkX = constrain(self.walkX, -0.8, 0.8)
        self.walkY = constrain(self.walkY, -0.6, 0.6)
        self.walkR = constrain(self.walkR, -0.6, 0.6)
        
        self.gaitStepN += self.gaitSpeed
        gSeqLength = len(self.gSeq)
        if self.gaitStepN >= gSeqLength: self.gaitStepN = 0
        
        # calculate rotation and translation of legs over ground  
        s = sin(-self.walkR / 100)
        c = cos(-self.walkR / 100)
        
        # move legs with ground
        for i in range(0, 6):
            if self.leg[i].z == 0:
                self.leg[i].x = (self.leg[i].x * c - self.leg[i].y * s) - self.walkX
                self.leg[i].y = (self.leg[i].y * c + self.leg[i].x * s) + self.walkY
                # move target within boundary to allow some movement without stepping 
                if self.leg[i].z == 0 and sqrt(sq(self.leg[i].x - self.legS[i].x) + sq(self.leg[i].y - self.legS[i].y)) < self.targetDeadZone:
                    self.legT[i].x = self.leg[i].x
                    self.legT[i].y = self.leg[i].y
                else: # reset target when getting to far off center
                    self.legT[i]._setto(self.legS[i])
                    
        # if leg's turn in sequence AND if not on target AND not already moving
        # TODO: Do not rely on rounding gaitStepN to match the sequence in gSeq
        for legN in range (0, 6):
            if self.gSeq[int(self.gaitStepN)][legN] == 1 and not self.leg[legN].equalegS(self.legT[legN]) and self.legMoving[legN] == 0:
                #println("gaitStepN: " + self.gaitStepN + " legN: " + self.legN)
                self.leg[legN].xt = self.legS[legN].x
                self.leg[legN].yt = self.legS[legN].y
                self.leg[legN].xs = self.leg[legN].x
                self.leg[legN].ys = self.leg[legN].y
                self.legMoving[legN] = 1
            
            # if leg ist already moving
            if self.legMoving[legN] == 1:
                distanceLeft = self.leg[legN].move2D(self.legSpeed); 
                #self.leg[legN].z = self.legS[legN].z + self.stepHeight-self.stepHeight*sq(2*distanceLeft-1) # quadratic function leg lift
                self.leg[legN].z = self.legS[legN].z + self.stepHeight / 2 * (sin(2 * PI * (distanceLeft - 0.25)) + 1) # smoother sinus leg lift
                if self.leg[legN].z == 0:
                    self.legMoving[legN] = 0

    # Body Translation/Rotation
    def runBodyIK(self):

        self.traX = constrain(self.traX, -40, 40)
        self.traY = constrain(self.traY, -40, 40)
        self.traZ = constrain(self.traZ, -40, 40)
        
        self.rotX = constrain(self.rotX, -0.3, 0.3)
        self.rotY = constrain(self.rotY, -0.25, 0.25)
        self.rotZ = constrain(self.rotZ, -0.25, 0.25)
        
        for i in range(0, 6):
            self.legIK[i].x = self.leg[i].x * cos(self.rotZ) * cos(self.rotX) - self.leg[i].z * cos(self.rotZ) * sin(self.rotX) + self.leg[i].y * sin(self.rotZ) + self.traX
            self.legIK[i].y = self.leg[i].x * (sin(self.rotY) * sin(self.rotX) - cos(self.rotY) * sin(self.rotZ) * cos(self.rotX)) + self.leg[i].z * (cos(self.rotY) * sin(self.rotZ) * sin(self.rotX) + sin(self.rotY) * cos(self.rotX)) + self.leg[i].y * cos(self.rotY) * cos(self.rotZ) + self.traY
            self.legIK[i].z = self.leg[i].x * (sin(self.rotY) * sin(self.rotZ) * cos(self.rotX) + cos(self.rotY) * sin(self.rotX)) + self.leg[i].z * (sin(self.rotY) * sin(self.rotZ) * sin(self.rotX) + cos(self.rotY) * cos(self.rotX)) - self.leg[i].y * sin(self.rotY) * cos(self.rotZ) + self.traZ

    # Full Leg IK (based on https://oscarliang.com/inverse-kinematics-implementation-hexapod-robots/)
    def runLegIK(self):
        mathError = 0
        for i in range(0, 6):
            deltaX = (self.legIK[i].x - self.coxaPos[i].x)
            deltaY = (self.legIK[i].y - self.coxaPos[i].y)
            deltaZ = -(self.legIK[i].z - self.coxaPos[i].z)
            
            legLength = sqrt(sq(deltaX) + sq(deltaY))
            HF = sqrt(sq(legLength - self.coxa)+sq(deltaZ))
            # Throw error if target is unreachable
            if (HF > self.femur + self.tibia) or (HF < abs(self.femur - self.tibia)): mathError = 1
            AX1 = atan((legLength - self.coxa)/deltaZ)
            if AX1 < 0: AX1 = PI + AX1
            AX2 = acos((sq(self.tibia)-sq(self.femur)-sq(HF))/(-2*self.femur*HF))
            self.legAngle[i][1] = PI / 2 - (AX1 + AX2)
            BX1 = acos((sq(HF)-sq(self.tibia)-sq(self.femur))/(-2*self.femur*self.tibia))
            self.legAngle[i][2] = PI / 2 - BX1
            self.legAngle[i][0] = atan(deltaY / deltaX)            
        return mathError
    
    
# derived from Oscar Liangs legged robot code base
class vector(object):

    x = float()
    y = float()
    z = float()
    xs = float()
    ys = float()
    zs = float()
    xt = float()
    yt = float()
    zt = float()
    oldmillis = 0

    #public:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

    def setto(self, nx, ny, nz):
        self.xs = self.xt = self.x = nx
        self.ys = self.yt = self.y = ny
        self.zs = self.zt = self.z = nz

    # different name instead of overloading in Java
    def _setto(self, v):
        self.xs = self.xt = self.x = v.x
        self.ys = self.yt = self.y = v.y
        self.zs = self.zt = self.z = v.z

    def equalegS(self, v):
        isEqual = True
        if self.x != v.x or self.y != v.y or self.z != v.z:
            isEqual = False
        return isEqual

    def move2D(self, pps):
        totalDistance = sqrt(sq(self.xt - self.xs) + sq(self.yt - self.ys))
        
        pps = 30 / pps
        dx = self.xt - self.x
        dy = self.yt - self.y
        distance = sqrt(dx * dx + dy * dy)
        if dx == 0: dx = 0.001 # avoid zero division
        angle = abs(atan(dy / dx))
        if distance > totalDistance / pps:
            if dx > 0: self.x += cos(angle) * totalDistance / pps
            else: self.x -= cos(angle) * totalDistance / pps
            if dy > 0: self.y += sin(angle) * totalDistance / pps
            else: self.y -= sin(angle) * totalDistance / pps
        else:
            self.x = self.xt
            self.y = self.yt
        distance = sqrt(sq(self.xt - self.x) + sq(self.yt - self.y))
        
        distanceLeft = distance / totalDistance
        if totalDistance == 0: distanceLeft = 0
        
        return distanceLeft
