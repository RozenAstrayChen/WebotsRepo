"""gripper controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Supervisor

# create the Robot instance.
#robot = Robot()

# get the time step of the current world.
# Initialize the Webots Supervisor.
supervisor = Supervisor()
timeStep = int(4 * supervisor.getBasicTimeStep())

# Initialize the arm motors.
finger_1 = []
for motorName in ['finger_1_joint_1', 'finger_2_joint_1', 'finger_middle_joint_1']:
    motor = supervisor.getMotor(motorName)
    motor.setVelocity(1.0)
    finger_1.append(motor)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
for motor in finger_1:
    motor.setPosition(1)

# Enter here exit cleanup code.
