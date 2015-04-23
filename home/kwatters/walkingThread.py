import threading

########################################
# The Walking Thread
# This is a thread that you can pass 
# an inmoov and a servo to.  It will
# start walking forward and animating in a loop
########################################
class WalkingThread(threading.Thread):
  # constructor for the thread, takes i01 and forwardServo
  def __init__(self,i01,forwardServo):
    super(WalkingThread, self).__init__()
    this.forwardServo = forwardServo
    this.i01 = i01
    # initially the thread is not running.
    this.running = False
  # The thread is started this method runs  
  def run(self):
    # flip the state to running
    this.running = True
    # move the servo to go forward
    forwardServo.moveTo(60)
    # while we are running, animate
    while this.running:
      i01.setHandSpeed("left", 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
      i01.setHandSpeed("right", 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
      i01.setArmSpeed("right", 0.95, 0.95, 0.95, 0.85)
      i01.setArmSpeed("left", 0.95, 0.95, 0.95, 0.85)
      i01.setHeadSpeed(0.75, 0.75)
      i01.moveHead(70,79,85,85,65)
      i01.moveArm("left",5,90,23,10)
      i01.moveArm("right",15,90,30,10)
      i01.moveHand("left",92,33,37,71,66,25)
      i01.moveHand("right",81,66,82,60,105,113)
      i01.moveTorso(75,97,90)
      sleep(2)
      i01.moveHead(79,100,85,85,65)
      i01.moveArm("left",15,84,36,15)
      i01.moveArm("right",5,82,22,20)
      i01.moveHand("left",92,33,37,71,66,25)
      i01.moveHand("right",81,66,82,60,105,113)
      i01.moveTorso(124,83,90)
      sleep(2)
    # we are no longer running, move servo and relax.  
    forwardServo.moveTo(93)
    relax()
 
 
walkingThread = WalkingThread(i01,forwardServo)
 
def heard(data):
  data = msg_i01_ear_recognized.data[0]
  if (data == "go forward"):
    # start the walking thread.
    walkingThread.start()
  if (data == "kill the motor"):
    # tell the thread to stop so it breaks out of the loop
    walkingThread.running = False
        