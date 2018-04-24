import numpy
from math import cos, sin
import time


class SvenzvaKinematics:

    NUM_JOINTS = 6
    ERROR_BAD_FORMAT = -1
    ERROR_NO_CONVERGE = -2
    ERROR_SINGULAR = -3

    def __init__(self):
        self.data = []
        self.NUM_JOINTS = 6
        self.T = numpy.zeros(shape=(6,6))
        self.det = 1.0
    """
    Solves the analytical jacobian given the joint position array q_ar
    Returns a numerical jacobian matrix
    """
    def solve_jacobian(self,q_ar):
        if not isinstance(q_ar, numpy.ndarray) or len(q_ar) != self.NUM_JOINTS:
            return SvenzvaKinematics.ERROR_BAD_FORMAT

        q1 = q_ar[0]
        q2 = q_ar[1]
        q3 = q_ar[2]
        q4 = q_ar[3]
        q5 = q_ar[4]
        q6 = q_ar[5]
        start = time.time()

        self.T[0][0] = sin(q1)*(-6.025E-3)-cos(q1)*cos(q2)*7.378496964862803E-19+cos(q1)*sin(q2)*2.662502E-1-cos(q4)*sin(q1)*2.31625E-2+sin(q1)*sin(q3)*2.743843197132029E-17-sin(q1)*sin(q4)*sin(q5)*1.4014E-1-cos(q1)*cos(q2)*cos(q4)*2.836588148525057E-18-cos(q1)*cos(q2)*sin(q3)*2.240518E-1+cos(q1)*cos(q3)*sin(q2)*2.240518E-1+cos(q3)*sin(q1)*sin(q4)*2.836588148525057E-18+cos(q5)*sin(q1)*sin(q3)*1.716220024325101E-17-cos(q1)*cos(q2)*cos(q3)*sin(q4)*2.31625E-2-cos(q1)*cos(q2)*cos(q5)*sin(q3)*1.4014E-1+cos(q1)*cos(q3)*cos(q5)*sin(q2)*1.4014E-1-cos(q1)*cos(q2)*sin(q4)*sin(q5)*1.716220024325101E-17-cos(q3)*cos(q4)*sin(q1)*sin(q5)*1.716220024325101E-17-cos(q1)*sin(q2)*sin(q3)*sin(q4)*2.31625E-2+cos(q1)*cos(q2)*cos(q3)*cos(q4)*sin(q5)*1.4014E-1+cos(q1)*cos(q4)*sin(q2)*sin(q3)*sin(q5)*1.4014E-1;
        self.T[0][1] = sin(q1)*(cos(q2)*4.320156490763342E36+sin(q2)*1.197229581605184E19+cos(q2)*cos(q3)*3.635448304028355E36+cos(q4)*sin(q2)*4.60262741641993E19+sin(q2)*sin(q3)*3.635448304028355E36+sin(q2)*sin(q4)*sin(q5)*2.784726200268058E20+cos(q2)*cos(q3)*cos(q5)*2.273901505484596E36-cos(q2)*sin(q3)*sin(q4)*3.758330499556655E35+cos(q3)*sin(q2)*sin(q4)*3.758330499556655E35+cos(q5)*sin(q2)*sin(q3)*2.273901505484596E36+cos(q2)*cos(q4)*sin(q3)*sin(q5)*2.273901505484596E36-cos(q3)*cos(q4)*sin(q2)*sin(q5)*2.273901505484596E36)*6.162975822039155E-38;
        self.T[0][2] = cos(q1)*cos(q3)*(-2.743843197132029E-17)-sin(q1)*sin(q2)*sin(q3)*2.240518E-1-cos(q1)*cos(q3)*cos(q5)*1.716220024325101E-17-cos(q2)*cos(q3)*sin(q1)*2.240518E-1+cos(q1)*sin(q3)*sin(q4)*2.836588148525057E-18-cos(q2)*cos(q3)*cos(q5)*sin(q1)*1.4014E-1-cos(q1)*cos(q4)*sin(q3)*sin(q5)*1.716220024325101E-17+cos(q2)*sin(q1)*sin(q3)*sin(q4)*2.31625E-2-cos(q3)*sin(q1)*sin(q2)*sin(q4)*2.31625E-2-cos(q5)*sin(q1)*sin(q2)*sin(q3)*1.4014E-1-cos(q2)*cos(q4)*sin(q1)*sin(q3)*sin(q5)*1.4014E-1+cos(q3)*cos(q4)*sin(q1)*sin(q2)*sin(q5)*1.4014E-1;
        self.T[0][3] = cos(q1)*sin(q4)*(-2.31625E-2)-cos(q1)*cos(q3)*cos(q4)*2.836588148525057E-18+cos(q1)*cos(q4)*sin(q5)*1.4014E-1+cos(q2)*sin(q1)*sin(q4)*2.836588148525057E-18-cos(q2)*cos(q3)*cos(q4)*sin(q1)*2.31625E-2-cos(q2)*cos(q4)*sin(q1)*sin(q5)*1.716220024325101E-17-cos(q1)*cos(q3)*sin(q4)*sin(q5)*1.716220024325101E-17-cos(q4)*sin(q1)*sin(q2)*sin(q3)*2.31625E-2-cos(q2)*cos(q3)*sin(q1)*sin(q4)*sin(q5)*1.4014E-1-sin(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*1.4014E-1;
        self.T[0][4] = cos(q1)*cos(q5)*sin(q4)*1.4014E-1+cos(q1)*sin(q3)*sin(q5)*1.716220024325101E-17+cos(q1)*cos(q3)*cos(q4)*cos(q5)*1.716220024325101E-17-cos(q2)*cos(q5)*sin(q1)*sin(q4)*1.716220024325101E-17+cos(q2)*sin(q1)*sin(q3)*sin(q5)*1.4014E-1-cos(q3)*sin(q1)*sin(q2)*sin(q5)*1.4014E-1+cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1)*1.4014E-1+cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3)*1.4014E-1;
        self.T[1][0] = cos(q1)*6.025E-3+cos(q1)*cos(q4)*2.31625E-2-cos(q2)*sin(q1)*7.378496964862803E-19-cos(q1)*sin(q3)*2.743843197132029E-17+sin(q1)*sin(q2)*2.662502E-1-cos(q2)*cos(q4)*sin(q1)*2.836588148525057E-18-cos(q1)*cos(q3)*sin(q4)*2.836588148525057E-18-cos(q1)*cos(q5)*sin(q3)*1.716220024325101E-17-cos(q2)*sin(q1)*sin(q3)*2.240518E-1+cos(q3)*sin(q1)*sin(q2)*2.240518E-1+cos(q1)*sin(q4)*sin(q5)*1.4014E-1+cos(q1)*cos(q3)*cos(q4)*sin(q5)*1.716220024325101E-17-cos(q2)*cos(q3)*sin(q1)*sin(q4)*2.31625E-2-cos(q2)*cos(q5)*sin(q1)*sin(q3)*1.4014E-1+cos(q3)*cos(q5)*sin(q1)*sin(q2)*1.4014E-1-cos(q2)*sin(q1)*sin(q4)*sin(q5)*1.716220024325101E-17-sin(q1)*sin(q2)*sin(q3)*sin(q4)*2.31625E-2+cos(q2)*cos(q3)*cos(q4)*sin(q1)*sin(q5)*1.4014E-1+cos(q4)*sin(q1)*sin(q2)*sin(q3)*sin(q5)*1.4014E-1;
        self.T[1][1] = cos(q1)*(cos(q2)*4.320156490763342E36+sin(q2)*1.197229581605184E19+cos(q2)*cos(q3)*3.635448304028355E36+cos(q4)*sin(q2)*4.60262741641993E19+sin(q2)*sin(q3)*3.635448304028355E36+sin(q2)*sin(q4)*sin(q5)*2.784726200268058E20+cos(q2)*cos(q3)*cos(q5)*2.273901505484596E36-cos(q2)*sin(q3)*sin(q4)*3.758330499556655E35+cos(q3)*sin(q2)*sin(q4)*3.758330499556655E35+cos(q5)*sin(q2)*sin(q3)*2.273901505484596E36+cos(q2)*cos(q4)*sin(q3)*sin(q5)*2.273901505484596E36-cos(q3)*cos(q4)*sin(q2)*sin(q5)*2.273901505484596E36)*(-6.162975822039155E-38);
        self.T[1][2] = cos(q3)*sin(q1)*(-2.743843197132029E-17)+sin(q1)*sin(q3)*sin(q4)*2.836588148525057E-18+cos(q1)*cos(q2)*cos(q3)*2.240518E-1-cos(q3)*cos(q5)*sin(q1)*1.716220024325101E-17+cos(q1)*sin(q2)*sin(q3)*2.240518E-1+cos(q1)*cos(q2)*cos(q3)*cos(q5)*1.4014E-1-cos(q1)*cos(q2)*sin(q3)*sin(q4)*2.31625E-2+cos(q1)*cos(q3)*sin(q2)*sin(q4)*2.31625E-2+cos(q1)*cos(q5)*sin(q2)*sin(q3)*1.4014E-1-cos(q4)*sin(q1)*sin(q3)*sin(q5)*1.716220024325101E-17+cos(q1)*cos(q2)*cos(q4)*sin(q3)*sin(q5)*1.4014E-1-cos(q1)*cos(q3)*cos(q4)*sin(q2)*sin(q5)*1.4014E-1;
        self.T[1][3] = sin(q1)*sin(q4)*(-2.31625E-2)-cos(q1)*cos(q2)*sin(q4)*2.836588148525057E-18-cos(q3)*cos(q4)*sin(q1)*2.836588148525057E-18+cos(q4)*sin(q1)*sin(q5)*1.4014E-1+cos(q1)*cos(q2)*cos(q3)*cos(q4)*2.31625E-2+cos(q1)*cos(q2)*cos(q4)*sin(q5)*1.716220024325101E-17+cos(q1)*cos(q4)*sin(q2)*sin(q3)*2.31625E-2-cos(q3)*sin(q1)*sin(q4)*sin(q5)*1.716220024325101E-17+cos(q1)*cos(q2)*cos(q3)*sin(q4)*sin(q5)*1.4014E-1+cos(q1)*sin(q2)*sin(q3)*sin(q4)*sin(q5)*1.4014E-1;
        self.T[1][4] = sin(q1)*sin(q3)*sin(q5)*1.716220024325101E-17+cos(q5)*sin(q1)*sin(q4)*1.4014E-1+cos(q1)*cos(q2)*cos(q5)*sin(q4)*1.716220024325101E-17+cos(q3)*cos(q4)*cos(q5)*sin(q1)*1.716220024325101E-17-cos(q1)*cos(q2)*sin(q3)*sin(q5)*1.4014E-1+cos(q1)*cos(q3)*sin(q2)*sin(q5)*1.4014E-1-cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5)*1.4014E-1-cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3)*1.4014E-1;
        self.T[2][1] = cos(q2)*(-7.378496964862803E-19)+sin(q2)*2.662502E-1-cos(q2)*cos(q4)*2.836588148525057E-18-cos(q2)*sin(q3)*2.240518E-1+cos(q3)*sin(q2)*2.240518E-1-sin(q2)*sin(q3)*sin(q4)*2.31625E-2-cos(q2)*cos(q3)*sin(q4)*2.31625E-2-cos(q2)*cos(q5)*sin(q3)*1.4014E-1+cos(q3)*cos(q5)*sin(q2)*1.4014E-1-cos(q2)*sin(q4)*sin(q5)*1.716220024325101E-17+cos(q2)*cos(q3)*cos(q4)*sin(q5)*1.4014E-1+cos(q4)*sin(q2)*sin(q3)*sin(q5)*1.4014E-1;
        self.T[2][2] = cos(q2)*sin(q3)*2.240518E-1-cos(q3)*sin(q2)*2.240518E-1+sin(q2)*sin(q3)*sin(q4)*2.31625E-2+cos(q2)*cos(q3)*sin(q4)*2.31625E-2+cos(q2)*cos(q5)*sin(q3)*1.4014E-1-cos(q3)*cos(q5)*sin(q2)*1.4014E-1-cos(q2)*cos(q3)*cos(q4)*sin(q5)*1.4014E-1-cos(q4)*sin(q2)*sin(q3)*sin(q5)*1.4014E-1;
        self.T[2][3] = sin(q2)*sin(q4)*2.836588148525057E-18+cos(q2)*cos(q4)*sin(q3)*2.31625E-2-cos(q3)*cos(q4)*sin(q2)*2.31625E-2-cos(q4)*sin(q2)*sin(q5)*1.716220024325101E-17+cos(q2)*sin(q3)*sin(q4)*sin(q5)*1.4014E-1-cos(q3)*sin(q2)*sin(q4)*sin(q5)*1.4014E-1;
        self.T[2][4] = sin(q2)*sin(q3)*sin(q5)*1.4014E-1+cos(q2)*cos(q3)*sin(q5)*1.4014E-1-cos(q5)*sin(q2)*sin(q4)*1.716220024325101E-17-cos(q2)*cos(q4)*cos(q5)*sin(q3)*1.4014E-1+cos(q3)*cos(q4)*cos(q5)*sin(q2)*1.4014E-1;
        self.T[3][1] = -cos(q1);
        self.T[3][2] = cos(q1)-cos(q2)*sin(q1)*1.224646799147353E-16;
        self.T[3][3] = cos(q1)*sin(q3)*1.224646799147353E-16+cos(q2)*sin(q1)*sin(q3)-cos(q3)*sin(q1)*sin(q2);
        self.T[3][4] = -cos(q1)*cos(q4)+cos(q2)*cos(q4)*sin(q1)*1.224646799147353E-16+cos(q1)*cos(q3)*sin(q4)*1.224646799147353E-16+cos(q2)*cos(q3)*sin(q1)*sin(q4)+sin(q1)*sin(q2)*sin(q3)*sin(q4);
        self.T[3][5] = sin(q5)*(cos(q1)*sin(q4)*4.056481920730334E31+cos(q1)*cos(q3)*cos(q4)*4.967757600021511E15-cos(q2)*sin(q1)*sin(q4)*4.967757600021511E15+cos(q2)*cos(q3)*cos(q4)*sin(q1)*4.056481920730334E31+cos(q4)*sin(q1)*sin(q2)*sin(q3)*4.056481920730334E31)*2.465190328815662E-32-cos(q5)*(cos(q1)*sin(q3)*4.967757600021511E15+cos(q2)*sin(q1)*sin(q3)*4.056481920730334E31-cos(q3)*sin(q1)*sin(q2)*4.056481920730334E31)*2.465190328815662E-32;
        self.T[4][1] = -sin(q1);
        self.T[4][2] = sin(q1)+cos(q1)*cos(q2)*1.224646799147353E-16;
        self.T[4][3] = sin(q1)*sin(q3)*1.224646799147353E-16-cos(q1)*cos(q2)*sin(q3)+cos(q1)*cos(q3)*sin(q2);
        self.T[4][4] = -cos(q4)*sin(q1)-cos(q1)*cos(q2)*cos(q4)*1.224646799147353E-16+cos(q3)*sin(q1)*sin(q4)*1.224646799147353E-16-cos(q1)*cos(q2)*cos(q3)*sin(q4)-cos(q1)*sin(q2)*sin(q3)*sin(q4);
        self.T[4][5] = sin(q5)*(sin(q1)*sin(q4)*4.056481920730334E31+cos(q1)*cos(q2)*sin(q4)*4.967757600021511E15+cos(q3)*cos(q4)*sin(q1)*4.967757600021511E15-cos(q1)*cos(q2)*cos(q3)*cos(q4)*4.056481920730334E31-cos(q1)*cos(q4)*sin(q2)*sin(q3)*4.056481920730334E31)*2.465190328815662E-32-cos(q5)*(sin(q1)*sin(q3)*4.967757600021511E15-cos(q1)*cos(q2)*sin(q3)*4.056481920730334E31+cos(q1)*cos(q3)*sin(q2)*4.056481920730334E31)*2.465190328815662E-32;
        self.T[5][0] = 1.0;
        self.T[5][2] = sin(q2)*(-1.224646799147353E-16);
        self.T[5][3] = cos(q2-q3);
        self.T[5][4] = cos(q4)*sin(q2)*1.224646799147353E-16-cos(q2)*sin(q3)*sin(q4)+cos(q3)*sin(q2)*sin(q4);
        self.T[5][5] = sin(q2)*sin(q4)*sin(q5)*(-1.224646799147353E-16)-cos(q2)*cos(q3)*cos(q5)-cos(q5)*sin(q2)*sin(q3)-cos(q2)*cos(q4)*sin(q3)*sin(q5)+cos(q3)*cos(q4)*sin(q2)*sin(q5);

        return 0



    """
    input:
        q_ar - numpyarray - [q0, ... qN] joint values for N joints
        ee_vel - numpyarray - desired end effector velocity in Twist format, ie [X,Y,Z,R,P,Y]

    output:
        joint velocity array
    """
    def get_joint_vel(self,q_ar, ee_vel):
        err = self.solve_jacobian(q_ar)
        if err != 0:
            return SvenzvaKinematics.ERROR_BAD_FORMAT
        if not isinstance(ee_vel, numpy.ndarray):
            return SvenzvaKinematics.ERROR_BAD_FORMAT

        self.det = numpy.linalg.det(self.T)
        if self.det != 0:
            try:
                R = numpy.linalg.inv(self.T).dot(ee_vel)
                return R
            except:
                return SvenzvaKinematics.ERROR_SINGULAR
        else:
            return SvenzvaKinematics.ERROR_SINGULAR

    """
    input:
        q_ar - numpyarray - [q0, ... , qN] joint values for N joints
        ee_vel - numpyarray - desired end effector velocity in Twist format
        time_step - double - time step to determine future joint value
    returns determinant of jacobian at the future joint position of the arm
    """
    def get_next_det(self, q_ar, ee_vel, time_step):
        err = self.solve_jacobian(q_ar)
        if err != 0:
            return SvenzvaKinematics.ERROR_BAD_FORMAT
        if not isinstance(ee_vel, numpy.ndarray):
            return SvenzvaKinematics.ERROR_BAD_FORMAT

        q_dot = numpy.linalg.inv(self.T).dot(ee_vel)
        for i, index in enumerate(q_ar):
            q_ar[i] += q_dot[i]*time_step

        old_det = self.det
        old_T = self.T
        err = self.solve_jacobian(q_ar)
        if err!= 0:
            return SvenzvaKinematics.ERROR_BAD_FORMAT
        res = numpy.linalg.det(self.T)
        self.T = old_T
        self.det = old_det

        return res



    def get_jacobian_det(self):
        return self.det

    def get_jacobian_wrist_det(self):
        return numpy.linalg.det(self.T[2:5, 2:5])

def test_case():
    kine = SvenzvaKinematics()
    q = numpy.array([0,0.2,0.2,0.2,0.2,0])
    twist = numpy.array([0,0,0,1,0,0])
    start = time.time()
    print kine.get_joint_vel(q,twist)
    print kine.get_jacobian_det()
    end = time.time()
    print "Took " + str(end - start) + "s"

if __name__== "__main__":
    test_case()
