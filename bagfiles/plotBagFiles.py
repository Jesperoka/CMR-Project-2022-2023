import rosbag
import matplotlib.pyplot as plt

# Which rosbag do you want to plot?
FILENAME = "bagfiles/trajectoryTrack.bag"

# Read data from bag
bag = rosbag.Bag(FILENAME)

# State published by the simulator
t = []
x = []
y = []
theta = []

# Inputs given by the controller
phi = []
v = []

# Reference Trajectory
xref = []
yref = []

# Read messages, append values and close rosbag
topics = ["/simulator_state", "/controller_input", "/FAKE_TOPIC"]
for topic, msg, timestamp in bag.read_messages(topics):
    if topic == topics[0] and len(msg.systemState) != 0: 
        t.append(msg.simulationTime)
        x.append(msg.systemState[0])
        y.append(msg.systemState[1])
        theta.append(msg.systemState[2])
  
    if topic == topics[1]:
        v.append(msg.controllerInput[0])
        phi.append(msg.controllerInput[1])
        
    if topic == topics[2]:
        xref.append(msg.data[0])
        yref.append(msg.data[1])
        
bag.close()

# Y vs X Plot
plt.figure(1)
plt.plot(x,y)
plt.plot(xref, yref, "-x")
plt.plot(x[0],y[0],'ro')
plt.plot(x[len(x)-1],y[len(x)-1],'rx')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.grid(True)

# Steering Input vs Time Plot
plt.figure(2)
plt.subplot(211)
plt.plot(t,phi)
plt.xlabel("Time [s]")
plt.ylabel("Steer act [rad/s]")
plt.grid(True)

# Velocity Input vs Time Plot
plt.subplot(212)
plt.plot(t,v)
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m/s]")
plt.grid(True)

# X vs Time Plot
plt.figure(3)
plt.subplot(311)
plt.plot(t,x)
plt.xlabel("Time [s]")
plt.ylabel("x [m]")
plt.grid(True)

# Y vs Time Plot
plt.subplot(312)
plt.plot(t,y)
plt.xlabel("Time [s]")
plt.ylabel("y [m]")
plt.grid(True)

# Theta vs Time Plot
plt.subplot(313)
plt.plot(t,theta)
plt.xlabel("Time [s]")
plt.ylabel("theta [rad]")
plt.grid(True)

plt.show()

input("Press enter to end (removes figures)...")