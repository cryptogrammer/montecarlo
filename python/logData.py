import time
from Scribbler2 import *

# Connect to the scribbler
# Set timeout to 0 to read instantly, non-blocking

fname = "log-%d.txt" % time.time()

s = Scribbler2('/dev/tty.Fluke2-0ACB-Fluke2',fname)

# Set timeout to zero
print 'Connected!'
s.setIRPower(140)
s.setForwardness(1)
# Create a list of commands
commands = []
# Command is a list [cmd, leftMotor, rightMotor, time]
# Setting motors to 200 will drive 
# forward with the fluke facing forward
commands.append([200, 200, 2.4])
commands.append([0,0,0.7])
commands.append([0,200,1.2])
commands.append([0,0,0.7])
commands.append([200, 200, 3.1])
commands.append([0,0,0.7])
commands.append([0,200,1.2])
commands.append([0,0,0.7])
commands.append([200, 200, 4.5])
commands.append([0,0,0.7])
commands.append([0,200,1.2])
commands.append([0,0,0.7])
commands.append([200, 200, 3])
commands.append([0,0,0.7])
print ("Start!")
for c in commands:
  start = time.time()
  s.setMotors(c[0],c[1])
  while (time.time() - start < c[2]):
    s.getObstacle(1)
    time.sleep(0.1) # Read sensors at 1Hz



