
import sys
import time
import tempfile
from controller import Supervisor
try:
    import ikpy
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, URDFLink
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

class UR5E():
    def __init__(self):
        self.robot = Supervisor()
        self.arm = self.robot.getSelf()
        self.timeStep = int(4 * self.robot.getBasicTimeStep())
        # Create the arm chain from the URDF
        self.armChain = Chain.from_urdf_file('ur5e.urdf')
        self.arm_motors = self.setJoint()
    '''def __init__(self):
        self.robot = Supervisor()
        self.arm = self.robot.getSelf()
        self.timeStep = int(4 * self.robot.getBasicTimeStep())
        filename = None
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
            filename = file.name
            file.write(self.robot.getUrdf().encode('utf-8'))
        self.armChain = Chain.from_urdf_file(filename)

        # Initialize the arm motors.
        self.arm_motors = []
        for link in self.armChain.links:
            if 'sensor' in link.name:
                motor = self.robot.getMotor(link.name.replace('_sensor', ''))
                motor.setVelocity(1.0)
                self.arm_motors.append(motor)'''
    def setJoint(self):

        base_joint = self.robot.getMotor("shoulder_pan_joint")
        joint1 = self.robot.getMotor("shoulder_lift_joint")
        joint2 = self.robot.getMotor("elbow_joint")
        joint3 = self.robot.getMotor("wrist_1_joint")
        joint4 = self.robot.getMotor("wrist_2_joint")
        joint5 = self.robot.getMotor("wrist_3_joint")

        return [base_joint, joint1, joint2, joint3, joint4, joint5]
        # set base position

    def simpleDemo(self, target='TARGET'):
        target = self.robot.getFromDef(target)

        while self.robot.step(self.timeStep) != -1:

            target_position = target.getPosition()
            arm_position = self.arm.getPosition()
            x = target_position[0] - arm_position[0]
            y = -(target_position[2] - arm_position[2])
            z = target_position[1] - arm_position[1]

            ik_results = self.armChain.inverse_kinematics([x, y, z])
            print(ik_results)
            # Actuate the 3 first arm motors with the IK results.
            for i in range(3):
                self.arm_motors[i].setPosition(ik_results[i+1])

ur5e = UR5E()
ur5e.simpleDemo()

