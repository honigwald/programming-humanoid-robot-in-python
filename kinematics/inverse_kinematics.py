'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''


from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = []
        # YOUR CODE HERE
        ### numerical solution with jacobian matrix
        lambda_ = 1
        max_step = 0.1

        target = matrix([transform])

        def inverse_kinematics(x_e, y_e, theta_e, theta):
            for i in range(1000):
                Ts = forward_kinematics(T0, l, theta)
                Te = matrix([from_trans(Ts[-1])]).T
                e = target - Te
                e[e > max_step] = max_step
                e[e < -max_step] = -max_step
                T = matrix([from_trans(i) for i in Ts[1:-1]]).T
                J = Te - T
                dT = Te - T
                J[0, :] = -dT[1, :]     # x
                J[1, :] = dT[0, :]      # y
                J[-1, :] = 1            # angular
                d_theta = lambda_ * pinv(J) * e
                theta += asarray(d_theta.T)[0]
                if  linalg.norm(d_theta) < 1e-4:
                    break
            return theta
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.keyframes = ([], [], [])  # the result joint angles have to fill in


    def from_trans(m):
        '''get x, y, theta from transform matrix'''
        return [m[0, -1], m[1, -1], np.atan2(m[1, 0], m[0, 0])]

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
