
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
        self.sup = Supervisor()

        self.timeStep = int(4 * self.sup.getBasicTimeStep())
        # Create the arm chain from the URDF
        self.armChain = Chain.from_urdf_file('ur5e.urdf')
        self.arm_motors = self.getARMJoint()
        self.grip_motors = self.getGripperJoint()
    '''def __init__(self):
        self.robot = Supervisor()
        self.arm = self.robot.getSelf()
        self.timeStep = int(4 * self.robot.getBasicTimeStep())
        filename = None
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
            filename = file.name
            file.write(self.robot.getUrdf().encode('utf-8'))
        self.armChain = Chain.from_urdf_file(filename)
        print(self.armChain)
        # Initialize the arm motors.
        self.arm_motors = []
        for link in self.armChain.links:
            if 'sensor' in link.name:
                motor = self.robot.getMotor(link.name.replace('sensor', 'motor'))
                motor.setVelocity(1.0)
                self.arm_motors.append(motor)'''
    def getARMJoint(self):

        base_joint = self.sup.getMotor("shoulder_pan_joint")
        base_joint.setVelocity(0.5)
        joint1 = self.sup.getMotor("shoulder_lift_joint")
        joint1.setVelocity(0.5)
        joint2 = self.sup.getMotor("elbow_joint")
        joint2.setVelocity(0.5)
        joint3 = self.sup.getMotor("wrist_1_joint")
        joint3.setVelocity(0.5)
        joint4 = self.sup.getMotor("wrist_2_joint")
        joint4.setVelocity(0.5)
        joint5 = self.sup.getMotor("wrist_3_joint")
        joint5.setVelocity(0.5)
        # send joint command
        '''
        base_joint.setPosition(0.2)
        joint1.setPosition(-1.8)
        joint2.setPosition(-1.6)
        joint3.setPosition(-1.32)
        joint4.setPosition(1.57)
        joint5.setPosition(0.2)
        '''
        return [base_joint, joint1, joint2, joint3, joint4, joint5]
        # set base position
    def getGripperJoint(self):

        finger_1_joint_1 = self.sup.getMotor('finger_1_joint_1')
        finger_2_joint_1 = self.sup.getMotor('finger_1_joint_2')
        finger_middle_joint_1 = self.sup.getMotor('finger_1_joint_3')

        return [finger_1_joint_1, finger_2_joint_1, finger_middle_joint_1]
    def simpleDemo(self, target="target2"):
        target = self.sup.getFromDef(target)
        arm = self.sup.getSelf()
        
        for motor in self.grip_motors:
            print(motor)
            motor.setPosition(1)

        #self.grip_motors[0].setPosition(1)
        '''
        self.arm_motors[0].setPosition(0.2)
        self.arm_motors[1].setPosition(-1.8)
        self.arm_motors[2].setPosition(-1.6)
        self.arm_motors[3].setPosition(-1.32)
        self.arm_motors[4].setPosition(1.57)
        self.arm_motors[5].setPosition(0.2)
'''
        '''
        while self.sup.step(self.timeStep) != -1:
            
            target_position = target.getPosition()
            arm_position = arm.getPosition()
            x = target_position[0] - arm_position[0]
            y = -(target_position[2] - arm_position[2])
            z = target_position[1] - arm_position[1]

            ik_results = self.armChain.inverse_kinematics([x, y, z])
            print(ik_results)
            # Actuate the 3 first arm motors with the IK results.
            for i in range(6):
                self.arm_motors[i].setPosition(ik_results[i+1])
        '''
ur5e = UR5E()
ur5e.simpleDemo()

