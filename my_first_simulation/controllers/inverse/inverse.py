# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Demonstration of inverse kinematics using the "ikpy" Python module."""

import sys
import time
try:
    import ikpy
    from ikpy.chain import Chain
    from ikpy.link import OriginLink, URDFLink
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

import math
from controller import Supervisor

if ikpy.__version__[0] < '3':
    sys.exit('The "ikpy" Python module version is too old. '
             'Please upgrade "ikpy" Python module to version "3.0" or newer with this command: "pip install --upgrade ikpy"')

# Create the arm chain.
# The constants below have been manually extracted from the Irb4600-40.proto file, looking at the HingeJoint node fields.
# The chain should contain the "E motor" bone, because this bone defines the hand position.
armChain = Chain(name='arm', links=[
    OriginLink(),
    URDFLink(
        name="A motor",
        bounds=[-3.1415, 3.1415],
        translation_vector=[0, 0, 0.159498],
        orientation=[0, 0, 0],
        rotation=[0, 0, 1]
    ),
    URDFLink(
        name="B motor",
        bounds=[-1.5708, 2.61799],
        translation_vector=[0.178445, -0.122498, 0.334888],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0]
    ),
    URDFLink(
        name="C motor",
        bounds=[-3.1415, 1.309],
        translation_vector=[-0.003447, -0.0267, 1.095594],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0]
    ),
    URDFLink(
        name="D motor",
        bounds=[-6.98132, 6.98132],
        translation_vector=[0.340095, 0.149198, 0.174998],
        orientation=[0, 0, 0],
        rotation=[1, 0, 0]
    ),
    URDFLink(
        name="E motor",
        bounds=[-2.18166, 2.0944],
        translation_vector=[0.929888, 0, 0],
        orientation=[0, 0, 0],
        rotation=[0, 1, 0]
    ),
])

# Initialize the Webots Supervisor.
supervisor = Supervisor()
timeStep = int(4 * supervisor.getBasicTimeStep())

# Initialize the arm motors.
motors = []
for motorName in ['A motor', 'B motor', 'C motor', 'D motor', 'E motor', 'F motor']:
    motor = supervisor.getMotor(motorName)
    motor.setVelocity(1.0)
    motors.append(motor)

# Get the arm and target nodes.
target = supervisor.getFromDef('TARGET')
arm = supervisor.getSelf()



print('start find work space')
# Loop 2: Move the arm hand to the target.
print('Move the yellow and black sphere to move the arm...')
while supervisor.step(timeStep) != -1:
    # Get the absolute postion of the target and the arm base.
    targetPosition = target.getPosition()
    armPosition = arm.getPosition()
    print(targetPosition)
    # Compute the position of the target relatively to the arm.
    # x and y axis are inverted because the arm is not aligned with the Webots global axes.
    x = targetPosition[0] - armPosition[0]
    y = - (targetPosition[2] - armPosition[2])
    z = targetPosition[1] - armPosition[1]+0.5

    # Call "ikpy" to compute the inverse kinematics of the arm.
    ikResults = armChain.inverse_kinematics([x, y, z])

    # Actuate the 3 first arm motors with the IK results.
    for i in range(3):
        motors[i].setPosition(ikResults[i+1])
        
        
       # y +- 0.7
       # x +- 0.3
