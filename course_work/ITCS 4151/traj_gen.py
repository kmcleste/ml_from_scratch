#!/usr/bin/env python2.7
import rospy
import tf_conversions
import tf2_ros
import copy
from prm.msg import PRMPath
from math import fmod, pi, sin, cos, atan2, sqrt, pow, acos, floor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped



l1 = 3.0
l2 = 3.0
l3 = 2.0

pub_joints = rospy.Publisher('joint_states', JointState, queue_size=10)

def pathCb(data):
    print('In pathCb')

    path = data.qs

    startQ = path[0].position
    pubJointState(startQ)

    #goalQ = goalIK[0]
    T = 3.0
   
    allPos = []
    allVel = []
    allAcc = []


    # For each trajectory segment
    for i in range(1,len(path)):
        print('i: %s' % i)
        tempIK = path[i].position
        print(tempIK)

        #print('tempIK: %s' % tempIK)

        targetQ = path[i].position
        #print('startQ: %s' % startQ)
        #print('targetQ: %s' % targetQ)
        #input('Press enter to continue')

        for q in range(len(startQ)):
            print('**************************')
            print(' Joint %s' % q)
            
            # Compute trajectory for first joint
            cc = cubCoef(startQ[q], targetQ[q], T)
            print('cc: %s' % cc)
            pos, vel, acc = trajec(T, 0.01, cc)
            print('len(pos): %s' % len(pos))
 

            allPos.append(pos) if i == 1 else allPos[q].extend(pos)
            allVel.append(vel) if i == 1 else allVel[q].extend(vel)
            allAcc.append(acc) if i == 1 else allAcc[q].extend(acc)
            print('len(allPos): %s' % len(allPos))
        
        startQ = copy.deepcopy(targetQ)

    print('**********************')
    #print(allPos[2])

    print(allPos)
    sendTrajToRviz(allPos, 0.01)


    print('Exiting pathCb')

# Returns a 2D list:
# [[solA], [solB]]
def solveTheta2And3(r, z):
    print('In solveTheta23')

    #******************************************************
    # theta2 + theta3
    #******************************************************
    A = -2.0*l3*r
    B = -2.0*l3*(z - l1)
    C = pow(r,2) + pow(z-l1,2) + pow(l3,2) - pow(l2,2)
    print('A: %s B: %s C: %s' % (A,B,C))
    print('-C/(sqrt(pow(A,2) + pow(B,2))): %s' % (-C/(sqrt(pow(A,2) + pow(B,2)))))

    psi = atan2(B,A)
    print('psi: %s' % psi)

    # Two pairs of values of theta2 and theta3 based on +-acos
    # Do fmod because we can end up with values outside of [-pi,pi]
    # because acos returns [-pi,pi] and psi can be [-pi,pi]
    theta23_a = fmod((acos( -C/(sqrt(pow(A,2) + pow(B,2))) ) + psi), pi)
    theta23_b = fmod((-acos( -C/(sqrt(pow(A,2) + pow(B,2))) ) + psi), pi)
    print('theta23_a: %s theta23_b: %s' % (theta23_a, theta23_b))


    #******************************************************
    # theta2
    #******************************************************
    theta2_a = atan2( z-l1 - (l3*sin(theta23_a)), r - (l3*cos(theta23_a)) )
    theta2_b = atan2( z-l1 - (l3*sin(theta23_b)), r - (l3*cos(theta23_b)) )
    print('theta2_b: "%s' % theta2_b)

    #******************************************************
    # theta3
    #******************************************************
    theta3_a = theta23_a - theta2_a
    theta3_b = theta23_b - theta2_b

    print('Exiting solveTheta23')
    return [[theta2_a, theta3_a], [theta2_b, theta3_b]]



def IK(eePose):
    print('In IK')

    x = eePose.position.x
    y = eePose.position.y
    z = eePose.position.z

    #******************************************************
    # theta1
    # theta 1 is based on an overhead view. z is a dot.
    # Two values for theta1 based on +r and -r
    #******************************************************
    r_a = sqrt( pow(x,2) + pow(y,2) )
    r_b = -r_a
    theta1_a = atan2( y/r_a, x/r_a )
    theta1_b = atan2( y/r_b, x/r_b )
    print('r_a: %s r_b: %s theta1_a: %s theta1_b: %s' % (r_a, r_b, theta1_a, theta1_b))


    # Use a function for the other theta values

    print('*************2nd SOL*****************')
    solsRA = solveTheta2And3(r_a, z)
    print('*************END 2nd SOL*****************')
    solsRB = solveTheta2And3(r_b, z)

    sol1 = [theta1_a, solsRA[0][0], solsRA[0][1]]
    sol2 = [theta1_a, solsRA[1][0], solsRA[1][1]]
    sol3 = [theta1_b, solsRB[0][0], solsRB[0][1]]
    sol4 = [theta1_b, solsRB[1][0], solsRB[1][1]]

    print('Exiting IK')
    return [sol1, sol2, sol3, sol4]


#May need to pass in something to decide which qg to use
#Needs to be one closest to previous
def cubCoef(qs, qg, T):
    print('In cubCoef')

    # theta(t) = a0 + a1t + a2t^2 + a3t^3

    '''
    Constraints:
    theta(0)=qs
    theta(T)=qg
    theta_dot(0)=0
    theta_dot(T)=0

    Equations:
    1 theta(t) = a0 + a1*t + a2*t^2 + a3*t^3
    2 theta_dot(t) = a1 + 2*a2*t + 3*a3*t^2
    3 theta_ddot(t) = 2*a2 + 6*a3*t
    '''
    
    # Plug the constraints into the equations
    # theta(0) = qs into eq 1
    a0 = qs

    # theta_dot(0) = 0 into eq 2
    a1 = 0.0

    # and these two....
    # theta_f = a0 + a1*T + a2*T^2 + a3*T^3
    # 0 = a1 + 2*a2*T + 3*a3*T^2

    # Solve the equations for a2 and a3
    # Use findAngle.. function because it wraps angles to be within [-pi,pi]
    a2 = (3.0/T**2) * findAngleBetweenAngles(qs,qg)
    a3 = (-2.0/T**3) * findAngleBetweenAngles(qs,qg)

    print('Exiting cubCoef')
    return [a0,a1,a2,a3]


# Creates a trajectory given the time and cubic coefficients
def trajec(t, resolution, cc):
    print('In trajec')

    num_steps = int(t / resolution)
    

    pos = []
    vel = []
    acc = []
    for i in range(num_steps):
        t = i*resolution

        pos_temp = cc[0] + cc[1]*t + cc[2]*t**2 + cc[3]*t**3
        vel_temp = cc[1] + 2.0*cc[2]*t + 3.0*cc[3]*t**2
        acc_temp = 2.0*cc[2] + 6.0*cc[3]*t

        pos.append(pos_temp)
        vel.append(vel_temp)
        acc.append(acc_temp)

    

    print('Exiting trajec')
    return pos, vel, acc



def pubJointState(thetas):

    # Create JointState object
    j = JointState()
    j.header.stamp = rospy.Time.now()
    j.name = ['joint1', 'joint2', 'joint3']
    j.position = thetas
    j.velocity = []
    j.effort = []


    # Publish it
    pub_joints.publish(j)


'''
Sends the trajectory positions to rviz at the specified time resolution

Parameters:
    allPos (list): A list of all the positions for each joint. 
                   The order of indices should be [joint][position].
    resolution (float): The elapsed time between each trajectory point
Returns:
    None
'''
def sendTrajToRviz(allPos, resolution):
    print('In sendTrajToRviz')

    rate = rospy.Rate(10)
    print(len(allPos[0]))
    T = len(allPos[0]) * resolution
    print('T: %s' % T)

    tNow = rospy.Time.now()
    tEnd = tNow + rospy.Duration(T)
    print('tNow: %s tEnd: %s' % (tNow.to_sec(), tEnd.to_sec()))

    while rospy.Time.now() < tEnd and not rospy.is_shutdown():
        thetas = []
        index = int((rospy.Time.now() - tNow).to_sec() / resolution)
        #print(index)
        for i in range(len(allPos)):
            thetas.append(allPos[i][index])
        
        pubJointState(thetas)
        rate.sleep()

    print('Exiting sendTrajToRviz')



def findAngleBetweenAngles(a, b):    
    result = 0
    difference = b - a
    
    if difference > pi:
      difference = fmod(difference, pi)
      result = difference - pi

    elif(difference < -pi):
      result = difference + (2.0*pi)

    else:
      result = difference

    return result


def wrapPi(ang):
    for i in range(len(ang)):
            
        if ang[i] < 0:
            print('In if')
            ang[i] = ang[i] % pi
    return ang



def getAngDistance(qStart, qEnd):

    result = 0.0

    for i in range(len(qStart)):
        result += abs(findAngleBetweenAngles(qStart[i], qEnd[i]))

    return result



def main():
    print('In main')
    rospy.init_node('traj_gen', anonymous=False)
    rospy.sleep(0.5)

    rospy.Subscriber('/prm_path', PRMPath, pathCb)

    rospy.spin()



if __name__ == '__main__':
    main()