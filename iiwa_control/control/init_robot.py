#!/usr/bin/env python
import sympy
import sympybotics
import numpy as np
import diff_kine_control_1
def init_robot():
    right_arm_robot_def = sympybotics.RobotDef('Right arm',  # robot name
                                           [('-pi/2', 0,101.115e-3, 'q'),
                                            ('pi/2', 0, 0, 'q'),
                                            ('pi/2', 0, 333.95e-3, 'q'),
                                            (0, 89.2e-3, 28.62e-3, 'q')],  # list of tuples with Denavit-Hartenberg parameters (alpha, a, d, theta)
                                            dh_convention = 'standard')  # either 'standard' or 'modified'
    right_arm_robot_def.frictionmodel = {'Coulomb',
                            'viscous'}  # options are None or a combination of 'Coulomb', 'viscous' and 'offset'
    right_arm_robot = sympybotics.RobotDynCode(right_arm_robot_def)

    test_q = [0.5, 0.5, 0.5, 0.5]
    # test_q = getAngPositions()
    q = right_arm_robot_def.q

    q_dict = dict(zip(q, test_q))

    J = right_arm_robot.kin.J[-1]
    T = right_arm_robot.geo.T[-1]
    diff_kine_control_1.control(J, T, q)


if __name__ == "__main__":
    init_robot()