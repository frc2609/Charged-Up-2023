import core
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
np.set_printoptions(precision=3)
np.set_printoptions(suppress=True)

def meters_to_inches(meters):
    return meters*39.3701

def inches_to_meters(inches):
    return inches/39.3701

def robot_angle_to_array(theta1, theta2, theta3):
    return np.array([math.radians(theta1), math.radians(180-theta2), meters_to_inches(theta3)])

def array_to_robot_angle(arr):
    assert isinstance(arr, np.ndarray)
    return np.array([math.degrees(arr[0]), math.degrees(math.radians(180)-arr[1]), inches_to_meters(arr[2])])

class ArmKinematics:
    def __init__(self) -> None:
        lower_pivot = 9
        lower_link_old = 21
        lower_link_new = 27
        upper_link = 25
        # how far away the next setpoint is allowed to be from the previous one
        self.angularTolerance = math.radians(10)#180)
        self.linearTolerance = inches_to_meters(2)#5)

        self.M_new = np.array([[1, 0,  0, (lower_link_new+upper_link)],
                               [0, 1,  0, 0],
                               [0, 0, 1, lower_pivot],
                               [0, 0,  0, 1]])
        self.Blist_new = np.array([[0, -1,  0,  0, 0, lower_link_new+upper_link],
                                   [0, 1,  0,  0, 0,   -(upper_link)],
                                   [0, 0, 0, 1, 0, 0]]).T
        self.Blist_new_noslider = np.array([[0, -1,  0,  0, 0, lower_link_new+upper_link],
                                   [0, 1,  0,  0, 0,   -(upper_link)]]).T

        self.M_old = np.array([[1, 0,  0, (lower_link_old+upper_link)],
                               [0, 1,  0, 0],
                               [0, 0, 1, lower_pivot],
                               [0, 0,  0, 1]])
        self.Blist_old = np.array([[0, -1,  0,  0, 0, lower_link_old+upper_link],
                                   [0, 1,  0,  0, 0,   -(upper_link)],
                                   [0, 0, 0, 1, 0, 0]]).T
        self.Blist_old_noslider = np.array([[0, -1,  0,  0, 0, lower_link_old+upper_link],
                                   [0, 1,  0,  0, 0,   -(upper_link)]]).T

        self.theta_zero = np.array([0, 0, 0])

    def old_to_new(self, T_old, joint_old, leaveSlider=False):
        if leaveSlider:
            joint_old_noslider = np.array([joint_old[0], joint_old[1]])
            oldPos_no_silder = core.FKinBody(self.M_old,self.Blist_old_noslider, joint_old_noslider)
            oldPos = core.FKinBody(self.M_old,self.Blist_old, joint_old)
            R, p = core.TransToRp(oldPos_no_silder)
            newPos_no_slider, success = core.IKinBody(self.Blist_new_noslider, self.M_new, oldPos_no_silder, joint_old_noslider, math.radians(90), inches_to_meters(2))
            if not success:
                # without leaveslider
                return (core.IKinBody(self.Blist_new, self.M_new, T_old, joint_old, self.angularTolerance, self.linearTolerance)[0], False)
            newPos_zero_slider = np.array([newPos_no_slider[0], newPos_no_slider[1], 0])
            return core.IKinBody(self.Blist_new, self.M_new, oldPos, newPos_zero_slider, self.angularTolerance, self.linearTolerance)

        return core.IKinBody(self.Blist_new, self.M_new, T_old, joint_old, self.angularTolerance, self.linearTolerance)
    
    def getJointValuesFromVector(self, v, guess):
        rot_guess, p_guess = core.TransToRp(self.getEEPos(guess))
        T = core.RpToTrans(rot_guess, v)
        print(rot_guess)
        print(math.degrees(guess[0]+guess[1]))
        guess_noslider = np.array(guess[:1])
        guess_noslider, success = core.IKinBody(self.Blist_new_noslider, self.M_new, T, guess_noslider, math.radians(360), inches_to_meters(25))
        if success:
            guess = np.array([guess_noslider[0], guess_noslider[1], 0])
            return core.IKinBody(self.Blist_new, self.M_new, T, guess, math.radians(360), self.linearTolerance)
        return (guess_noslider, False)

                      
    def getEEPos(self, theta, isOld=False):
        if(isOld):
            return core.FKinBody(self.M_old, self.Blist_old, theta)
        return core.FKinBody(self.M_new, self.Blist_new, theta)

    def getJointValues(self, T, guess, isOld=False):
        if isOld:
            return core.IKinBody(self.Blist_old, self.M_old, T, guess, self.angularTolerance, self.linearTolerance)
        return core.IKinBody(self.Blist_new, self.M_new, T, guess, self.angularTolerance, self.linearTolerance)

    def getJointSetpointsFromTrajectory(self, traj, initial_pose):
        theta = []
        # print(traj)
        guess =initial_pose
        for setp in traj:
            joint, success = self.getJointValues(setp, guess)
            if not success:
                raise NotImplemented
            theta.append(array_to_robot_angle(joint))
            guess = joint
        return theta

    def getEECoordinatesFromTrajectory(self, traj):
        x = []
        z = []
        # print(traj)
        for setp in traj:
            rot, setp = core.TransToRp(setp)
            x.append(setp[0])
            z.append(setp[2])
        return (x,z)

def printTrajectory(joint_MP, X, Z):
    # Display data
    print("time,lower,upper,extension,x,y")
    time = 0
    theta_1 = []
    theta_2 = []
    theta_3 = []
    t = []
    thickness = [1]

    for i in range(len(X)):
        time += 0.05
        t.append(time)
        joint_theta = joint_MP[i]
        # print(str(time) + ","+ str(joint_theta[0]) + "," + str(joint_theta[1]) + "," + str(joint_theta[2]) +"," +str(X[i]) + "," + str(Z[i]))

        theta_1.append(joint_theta[0])
        theta_2.append(joint_theta[1])
        if(joint_theta[2]<0):
            if(abs(joint_theta[2])<arm.linearTolerance):
                joint_theta[2] = 0
                theta_3.append(joint_theta[2])
            else:
                print("PANIC")
                print(joint_theta[2])
                raise NotImplemented
        else:
            theta_3.append(joint_theta[2])
        if(i>0):
            thickness.append(math.sqrt((X[i-1]+X[i])**2+(Z[i-1]+Z[i])**2))


        print("{" + "{:.4f}".format(joint_theta[0]) + "," + "{:.4f}".format(joint_theta[1]-90) + "," + "{:.4f}".format(joint_theta[2]) + "},")#+"," +str(X[i]) + "," + str(Z[i]))
        
    #print(theta_1)
    #print(theta_2)
    ## print([0.0]*30)
    #print(theta_3)
    points = np.array([X, Z]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segments, linewidths=(thickness/np.linalg.norm(np.array(thickness)))*30,color='blue')

    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2)
    ax1.add_collection(lc)
    ax1.set_xlim(0, max(X))
    ax1.set_ylim(0, max(Z))
    ax2.plot(t, theta_1)
    ax3.plot(t, theta_2)
    # print(len(theta_3))
    ax4.plot(t, theta_3)

    # fig,a = plt.subplots()
    # a.add_collection(lc)
    # a.set_xlabel("Distance away from the robot in inches")
    # a.set_ylabel("Distance above the ground in inches")
    # a.set_title("End effector position relative to the ground in the middle of the robot")
    # a.set_xlim(0, max(X))
    # a.set_ylim(0, max(Z))

    # print([0]*len(theta_1))
    plt.show()

# e.g. stow, inter, high, 0.5, 15, 1.5, 30
def generateTwoStepTrajectory(start, mid, end, time_1, num_points_1, time_2, num_points_2):
    # Convert to ee positions
    start_ee = arm.getEEPos(start)
    mid_ee = arm.getEEPos(mid)
    end_ee = arm.getEEPos(end)
    # Generate ee trajectories
    start_mid_ee_traj = core.ScrewTrajectory(start_ee, mid_ee, time_1, num_points_1, 3)
    mid_end_ee_traj = core.ScrewTrajectory(mid_ee, end_ee, time_2, num_points_2, 3)
    # Convert to arm positions
    start_mid_joint_MP = arm.getJointSetpointsFromTrajectory(start_mid_ee_traj, start)
    mid_end_joint_MP = arm.getJointSetpointsFromTrajectory(mid_end_ee_traj, mid)
    joint_MP = start_mid_joint_MP + mid_end_joint_MP
    # Graphing Calculations
    X, Z = arm.getEECoordinatesFromTrajectory(start_mid_ee_traj)
    X_1, Z_1 = arm.getEECoordinatesFromTrajectory(mid_end_ee_traj)
    X += X_1
    Z += Z_1
    printTrajectory(joint_MP, X, Z)

# e.g. stow, pickup, 1, 20
def generateTrajectory(start, end, time, num_points):
    start_ee = arm.getEEPos(start)
    end_ee = arm.getEEPos(end)
    ee_traj = core.ScrewTrajectory(start_ee, end_ee, time, num_points, 3)
    joint_MP = arm.getJointSetpointsFromTrajectory(ee_traj, start)
    X, Z = arm.getEECoordinatesFromTrajectory(ee_traj)
    printTrajectory(joint_MP, X, Z)

arm = ArmKinematics()
straight_rot = np.array([[1,0,0],[0,1,0], [0,0,1]])

# Positions
low = robot_angle_to_array(78.6, 63.7, 0.0)
stow = robot_angle_to_array(104.60, 21.09, 0.0)
mid = robot_angle_to_array(63.6, 139.7, 0.0)
inter = robot_angle_to_array(80, 90, 0)# 99.5, 94.00, 0.0)
high = robot_angle_to_array(57.4, 150.0, 0.44)

## GENERATE PATHS HERE
# generateTwoStepTrajectory(stow, inter, high, 0.5, 15, 1.0, 30)

# generateTrajectory(stow, inter, 0.5, 15)
# generateTrajectory(inter, high, 1.0, 30)

# generateTrajectory(stow, high, 1.5, 50)