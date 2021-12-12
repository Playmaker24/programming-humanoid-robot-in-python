'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    1. the local_trans has to consider different joint axes and link parameters for different joints
    2. Please use radians and meters as unit.
'''

# add PYTHONPATH
import os
import sys
import numpy as np
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity

from angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {
                       # YOUR CODE HERE
                       'Head': ['HeadYaw', 'HeadPitch'],
                       'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],
                       'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'],
                       'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
                       'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll'],
                       }
        self.axis = {
                    'Roll' : ['LShoulderRoll', 'LElbowRoll', 'LHipRoll', 'LAnkleRoll', 'RHipRoll', 'RAnkleRoll', 'RShoulderRoll',                                         'RElbowRoll'],
                    'Yaw'  : ['HeadYaw', 'LElbowYaw', 'RElbowYaw'],
                    'Pitch': ['HeadPitch', 'LShoulderPitch', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'RHipPitch', 'RKneePitch',                                         'RAnklePitch', 'RShoulderPitch'],
                    'YawPitch' : ['LHipYawPitch', 'RHipYawPitch']
                    }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def joint_offset(self, name):
        joint_offset_dict = {
            #Head
            'HeadYaw' : [0.0, 0.0, 126.5],
            'HeadPitch' : [0.0, 0.0, 0.0],
            #Arm
            'LShoulderPitch' : [0.0, 98.0, 100.0],
            'LShoulderRoll' : [0.0, 0.0, 0.0],
            'LElbowYaw' : [105.0, 15.0, 0.0],
            'LElbowRoll' : [0.0, 0.0, 0.0],
            'RShoulderPitch' : [0.0, -98.0, 100.0],
            'RShoulderRoll' : [0.0, 0.0, 0.0],
            'RElbowYaw' : [105.0, -15.0, 0.0],
            'RElbowRoll' : [0.0, 0.0, 0.0],
            #Legs
            'LHipYawPitch' : [0.0, 50.0, -85.0],
            'LHipRoll' : [0.0, 0.0, 0.0],
            'LHipPitch' : [0.0, 0.0, 0.0],
            'LKneePitch' : [0.0, 0.0, -100.0],
            'LAnklePitch' : [0.0, 0.0, -102.9],
            'LAnkleRoll' : [0.0, 0.0, 0.0],
            'RHipYawPitch' : [0.0, -50.0, -85.0],
            'RHipRoll' : [0.0, 0.0, 0.0],
            'RHipPitch' : [0.0, 0.0, 0.0],
            'RKneePitch' : [0.0, 0.0, -100.0],
            'RAnklePitch' : [0.0, 0.0, -102.9],
            'RAnkleRoll' : [0.0, 0.0, 0.0],
        }
            
        return joint_offset_dict.get(name)
    
    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)
        # YOUR CODE HERE
     
        #Rotation angle
        cosA = np.cos(joint_angle)
        sinA = np.sin(joint_angle)
        
        #Offset of link between joint
        lst = self.joint_offset(joint_name)
        dx = lst[0]
        dy = lst[1]
        dz = lst[2]
        
        # Roll x-Axis
        if joint_name in self.axis.get('Roll'):
            T = matrix([[1,   0,    0,   dx],
                        [0, cosA, -sinA, dy],
                        [0, sinA,  cosA, dz],
                        [0,   0,     0,   1]])
        # Pitch y-Axis         
        elif joint_name in self.axis.get('Pitch'):
            T = matrix([[cosA,  0, sinA, dx],
                        [  0,   1,  0,   dy],
                        [-sinA, 0, cosA, dz],
                        [  0,   0,  0,   1]])
        # Yaw z-Axis         
        elif joint_name in self.axis.get('Yaw'):
            T = matrix([[cosA,  sinA, 0, dx],
                        [-sinA, cosA, 0, dy],
                        [  0,    0,   1, dz],
                        [  0,    0,   0,  1]])                     
        # YawPitch yz-Axis, first Pitch then Yaw
        elif joint_name in self.axis.get('YawPitch'):
            #offset matrix
            TR = matrix([[1, 0, 0,  dx],
                         [ 0, 1, 0, dy],
                         [ 0, 0, 1, dz],
                         [ 0, 0, 0,  1]])
            #rotation on Pitch y-Axis
            R_y = matrix([[cosA,  0, sinA, 0],
                          [  0,   1,  0,   0],
                          [-sinA, 0, cosA, 0],
                          [  0,   0,  0,   1]])
            #rotation on Yaw z-Axis
            R_z = matrix([[cosA,  sinA, 0,  0],
                          [-sinA, cosA, 0,  0],
                          [  0,    0,   1,  0],
                          [  0,    0,   0,  1]])
            T = np.matmul(TR, np.matmul(R_z, R_y))
                              
        else: 
             pass
            
        #print(joint_angle)
        #print(joint_name)
        #print(T)
        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = np.dot(T,Tl)
                 
                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
