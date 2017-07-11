import SimpleOpenNI.ContextWrapper as ContextWrapper
import SimpleOpenNI.SimpleOpenNI as SimpleOpenNI
import SimpleOpenNI.SimpleOpenNIConstants as SimpleOpenNIConstants
from org.myrobotlab.service import OpenNi
import org.myrobotlab.openni.PMatrix3D as PMatrix3D
import org.myrobotlab.openni.PVector as PVector
import org.myrobotlab.openni
import threading
from threading import Thread as Thread
import time
import math
#to use this code you have to execute initArmNi before.

def attachArm():
	inmoov.rightArm.bicep.attach("inmoov.right",2)
	inmoov.rightArm.rotate.attach("inmoov.right",3)
	inmoov.rightArm.shoulder.attach("inmoov.right",4)
	inmoov.rightArm.omoplate.attach("inmoov.right",5)

def detachArm():
	inmoov.rightArm.bicep.detach()
	inmoov.rightArm.rotate.detach()
	inmoov.rightArm.omoplate.detach()
	inmoov.rightArm.shoulder.detach()

def positionRepo():
	inmoov.rightArm.bicep.moveTo(0)
	inmoov.rightArm.rotate.moveTo(100)
	inmoov.rightArm.omoplate.moveTo(90)
	inmoov.rightArm.shoulder.moveTo(90)

#variables used to get kinect data declaration.
rShoulderPosition = PVector()
lShoulderPosition = PVector()
rShoulderOrientation = PMatrix3D()
rElbowPosition = PVector()
headPosition = PVector()
torsoPosition = PVector()
lHipPosition = PVector()
rHipPosition = PVector()
rHandPosition = PVector()


#------------------------------------------------------------Function used for calculus----------------------------------------------------

def angle(a,b,c):
	angle01 = math.atan2(a.y - b.y, a.x - b.x)
	angle02 = math.atan2(b.y - c.y, b.x - c.x)
	return (angle01 - angle02)

def projectToBodyReferential(A,B, vecHip1, vecHip2):
	normeA = math.sqrt(math.pow(A.x - B.x, 2) + math.pow(A.y - B.y, 2) + math.pow(A.z - B.z, 2))
	normeVecHip = math.sqrt(math.pow(vecHip1.x - vecHip2.x, 2) + math.pow(vecHip1.y - vecHip2.y, 2) + math.pow(vecHip1.z - vecHip2.z, 2))
	newAx = (((A.x - B.x)/normeA) * ((vecHip1.x - vecHip2.x)/normeVecHip) + ((A.z - B.z)/normeA) * ((vecHip1.z - vecHip2.z)/normeVecHip))
	newAy = (A.y - B.y)
	newAz = (((A.x - B.x)/normeA) * ((vecHip1.z - vecHip2.z)/normeVecHip) + ((A.z - B.z)/normeA) * ((vecHip1.x - vecHip2.x)/normeVecHip))
	return PVector(newAx,newAy,newAz)

def myround(x, base):
	return int(base * round(float(x)/base))
	
def angleBetween(v1, v2):
    dot = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z
    v1mag = math.sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z)
    v2mag = math.sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z)
    return (float)(dot / (v1mag * v2mag))

#------------------------------------------------------------------------------------------------------------------------------------------
#rounding is the variable to round value : if rounding = 2 => the values will be 10 12 14 16 18 20....
#rounding is used to ignore noise on values.
rounding = 1


#------------------------------------------------------------Functions getting arm position------------------------------------------------

#Function getting from kinect the value for the movement made by the rotate motor.
def rotateToMotor(rElbowPosition, rShoulderPosition, rShoulderOrientation, homoplate):
	repereBicep = PMatrix3D(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1)
	y = PVector(0,1,0)
	normalY = PVector(1,1,1)
	rShoulderOrientation.mult(y,normalY)
	
	vectBicep = PVector(rShoulderPosition.x - rElbowPosition.x, rShoulderPosition.y - rElbowPosition.y, rShoulderPosition.z - rElbowPosition.z)
	repereBicepX = PVector(repereBicep.m00, repereBicep.m10, repereBicep.m20)
	repereBicepY = PVector(repereBicep.m01, repereBicep.m11, repereBicep.m21)
	repereBicepZ = PVector(repereBicep.m02, repereBicep.m12, repereBicep.m22)
	repereBicepAxis = [repereBicepX, repereBicepY, repereBicepZ]

	i = 2
	u = repereBicepAxis[i].cross(vectBicep)
	det = PMatrix3D(vectBicep.x, vectBicep.y, vectBicep.z, 0, repereBicepAxis[i].x, repereBicepAxis[i].y, repereBicepAxis[i].z, 0, u.x, u.y, u.z, 0, 0, 0, 0, 1).determinant()
	rotateAxisU = math.copysign(PVector.angleBetween(vectBicep, repereBicepAxis[i]), -det)
	repereBicep.rotate(rotateAxisU, u.x, u.y, u.z)

	repereBicepX = PVector(repereBicep.m00, repereBicep.m10, repereBicep.m20)
	repereBicepY = PVector(repereBicep.m01, repereBicep.m11, repereBicep.m21)
	repereBicepZ = PVector(repereBicep.m02, repereBicep.m12, repereBicep.m22)
	repereBicepAxis = [repereBicepX, repereBicepY, repereBicepZ]
	value = 180 - int((angleBetween(normalY, repereBicepY)*30+18)*3.8)
	value = int(value + homoplate/2)
	#print "test normalY bicepY is : "+str(value)
	if value < 0:
		return 0
	elif value > 180:
		return 180
	else:return myround(value, 5)

#Function getting from kinect the value for the movement made by the omoplate motor.
def homoplateToMotor(rShoulderPosition, rElbowPosition, lHipPosition, rHipPosition):
	value = projectToBodyReferential(rShoulderPosition, rElbowPosition, lHipPosition, rHipPosition).x
	value = (value*100-20)*(1.3)+60
	#value = (value*100-20)*(1.1)+70
	if value < 90:
		return 90
	elif value > 180:
		return 180
	else: return myround(value, rounding)

#Function getting from kinect the value for the movement made by the shoulder motor.
def shoulderToMotor(rElbowPosition, rShoulderPosition, rShoulderOrientation):
	repere = PMatrix3D(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1)
	x = PVector(1,0,0)
	normalX = PVector(1,1,1)
	rShoulderOrientation.mult(x,normalX)
	repereZ = PVector(repere.m02, repere.m12, repere.m22)
	value = 180-int(((angleBetween(normalX, repereZ)+1)*10)*9.5)
	if value < 0:
		return 0
	elif value > 180:
		return 180
	else:return myround(value, rounding)

#Function getting from kinect the value for the movement made by the bicep motor.
def bicepToMotor(vectBicep, vectForeArm):
	value = angleBetween(vectBicep, vectForeArm)*100+90
	if value < 0 :
		return 0
	elif value > 90 :
		return 90
	else:return myround(value, 1)

#------------------------------------------------------------------------------------------------------------------------------------------

#function called by the scheduler to get the joint's data and set the motor to the right value
def updatePosition(myWrapper, user):
	if myWrapper.isTrackingSkeleton(user):
		lHip = SimpleOpenNIConstants.SKEL_LEFT_HIP
		myWrapper.getJointPositionSkeleton(user,lHip ,lHipPosition)
		
		rHip = SimpleOpenNIConstants.SKEL_RIGHT_HIP
		myWrapper.getJointPositionSkeleton(user,rHip ,rHipPosition)
		
		head = SimpleOpenNIConstants.SKEL_HEAD
		myWrapper.getJointPositionSkeleton(user,head ,headPosition)
		
		torso = SimpleOpenNIConstants.SKEL_TORSO
		myWrapper.getJointPositionSkeleton(user,torso ,torsoPosition)
		
		rShoulder = SimpleOpenNIConstants.SKEL_RIGHT_SHOULDER
		myWrapper.getJointPositionSkeleton(user,rShoulder ,rShoulderPosition)
		probaShoulderOrientation = myWrapper.getJointOrientationSkeleton(user,rShoulder ,rShoulderOrientation)
		
		lShoulder = SimpleOpenNIConstants.SKEL_LEFT_SHOULDER
		myWrapper.getJointPositionSkeleton(user,lShoulder ,lShoulderPosition)
		
		rElbow = SimpleOpenNIConstants.SKEL_RIGHT_ELBOW
		probaElbowPosition = myWrapper.getJointPositionSkeleton(user,rElbow ,rElbowPosition)
		
		rHand = SimpleOpenNIConstants.SKEL_RIGHT_HAND
		probaHandPosition = myWrapper.getJointPositionSkeleton(user,rHand ,rHandPosition)
		
		
		vectBicep = PVector(rElbowPosition.x - rShoulderPosition.x, rElbowPosition.y - rShoulderPosition.y, rElbowPosition.z - rShoulderPosition.z)
		vectForeArm = PVector(rElbowPosition.x - rHandPosition.x, rElbowPosition.y - rHandPosition.y, rElbowPosition.z - rHandPosition.z)
		
		if probaShoulderOrientation >= 0.5 and probaHandPosition >= 0.5 and probaElbowPosition >= 0.5:
			try:
				bicep = bicepToMotor(vectBicep, vectForeArm)
				homoplate = homoplateToMotor(rShoulderPosition, rElbowPosition, lHipPosition, rHipPosition)
				rotate = rotateToMotor(rElbowPosition, rShoulderPosition, rShoulderOrientation, homoplate)
				shoulder = shoulderToMotor(rElbowPosition, rShoulderPosition, rShoulderOrientation)
				#print "angle bicep is : "+str(int(bicep))
				inmoov.rightArm.bicep.moveTo(int(bicep))
				#print "angle rotate : "+str(int(rotate))
				inmoov.rightArm.rotate.moveTo(int(rotate))
				#print "angle homoplate : "+str(homoplate)
				inmoov.rightArm.omoplate.moveTo(int(homoplate))
				#print "angle shoulder : "+str((shoulder))
				inmoov.rightArm.shoulder.moveTo(shoulder)
				print ""
			except Exception:
				pass
	else:
		print "user lost"

#function used to itialise simpleOpenNi wrapper.
#The wrapper is the object allowing us to use simpleOpenNi
def init():
	print "start initialise"
	users = []
	image = []
	myWrapper = SimpleOpenNI(ni)
	myWrapper.setMirror(True)
	myWrapper.enableIR(640, 640, 30)
	myWrapper.enableDepth(640, 640, 30)
	myWrapper.enableUser()
	print "initialised : "+str(myWrapper.isInit())
	if myWrapper.isInit():
		return myWrapper
	else:
		return Exception

#Scheduler is finding someone and the calling the update position with period specified
def schedule(myWrapper, duree, periode):
	print "start schedule"
	i = 1
	userFound = False
	user = False
	while i <= 100:
		try:
			user = myWrapper.getUsers()[0]
			myWrapper.startTrackingSkeleton(user)
			print "user found"
			userFound = True
			i = 101
		except Exception:
			myWrapper.update()
			i=i+1
			print i
			sleep(0.1)
	if i >= 100 and userFound == False:
		print "failed to find user"
		return
		
	startingTime = time.time()
	lastTime = time.time() - startingTime
	while (time.time() - startingTime) < duree: 
		if (time.time() - lastTime - startingTime) > periode:
			updatePosition(myWrapper, user)
			myWrapper.update()
			lastTime = time.time() - startingTime
	myWrapper.stopTrackingSkeleton(user)
	#myWrapper.close()
	print "stop schedule"

inmoov.rightArm.omoplate.setMinMax(90,180)
inmoov.rightArm.setVelocity(-1,-1,-1,-1)
#sleep(0.1)
#positionRepo()
#attachArm()
#sleep(2)
a = init()
#sleep(2)
#schedule(a, 10, 0.001)
#sleep(0.1)
#positionRepo()
#sleep(4)
#detachArm()
#print "stop"

#------------------------------------------------------------------------------------------------------------------------------------------
#-----------------------------------------------------------Using udp to control the code--------------------------------------------------

import socket
import threading
from threading import Thread as Thread

UDP_IP = "127.0.0.1"
UDP_PORT = 5019

#receiver, if receive track start tracking
def receive(ip, port, myWrapper):
	living = True
	UDP_IP = ip
	UDP_PORT = port
	
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.bind((UDP_IP, UDP_PORT))
	while living:
		print "is working"
		data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
		print "received message:", data
		if data == "stop":
			living = False
		elif data == "track":
			print "start track"
			positionRepo()
			attachArm()
			sleep(2)
			schedule(myWrapper, 30, 0.001)
			sleep(0.1)
			positionRepo()
			sleep(4)
			detachArm()
			print "stop track"

Thread(target=lambda: receive(UDP_IP,UDP_PORT, a)).start()

