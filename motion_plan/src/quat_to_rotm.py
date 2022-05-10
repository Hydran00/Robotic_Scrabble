import numpy as np
 
def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

x = np.zeros((3,3))
x=quaternion_rotation_matrix(np.array([0.707,0,0.707,0]))
y = np.zeros((4,4))
y[0][0]= x[0][0]
y[0][1]= x[0][1]
y[0][2]= x[0][2]
y[1][0]= x[1][0]
y[1][1]= x[1][1]
y[1][2]= x[1][2]
y[2][0]= x[2][0]
y[2][1]= x[2][1]
y[2][2]= x[2][2]

y[3,3] = 1

point = np.array([0.000001,0.0000001,0.00000001,1])

j = y.dot(point)
print(j)

y[0][0]= 1
y[0][1]= 0
y[0][2]= 0
y[0][3]= -0.15

y[1][0]= 0
y[1][1]= 1
y[1][2]= 0
y[1][3]= 0.2

y[2][0] = 0
y[2][1] = 0
y[2][2] = 1
y[2][3] = 0.986

y[3][0] = 0 
y[3][1] = 0
y[3][2] = 0
y[3][3] = 1

j = y.dot(j)

print(j)
