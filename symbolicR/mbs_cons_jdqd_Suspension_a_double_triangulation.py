#
#	MBsysTran - Release 8.1
#
#	Copyright 
#	Universite catholique de Louvain (UCLouvain) 
#	Mechatronic, Electrical Energy, and Dynamic systems (MEED Division) 
#	2, Place du Levant
#	1348 Louvain-la-Neuve 
#	Belgium 
#
#	http://www.robotran.be 
#
#	==> Generation Date: Tue Feb 17 12:09:53 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Suspension_a_double_triangulation
#
#	==> Number of joints: 4
#
#	==> Function: F18 - Constraints Quadratic Velocity Terms (Jdqd)
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def cons_jdqd(Jdqd, s):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S1 = sin(q[1])
    C1 = cos(q[1])
    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

 
# Constraints Quadratic Terms

    RLjdqd1_12 = s.dpt[1,5]*C1
    RLjdqd1_32 = -s.dpt[1,5]*S1
    ORjdqd1_12 = RLjdqd1_32*qd[1]
    ORjdqd1_32 = -RLjdqd1_12*qd[1]
    Apqpjdqd1_12 = ORjdqd1_32*qd[1]
    Apqpjdqd1_32 = -ORjdqd1_12*qd[1]
    ROjdqd2_72 = C2*S3+S2*C3
    ROjdqd2_92 = C2*C3-S2*S3
    RLjdqd2_12 = s.dpt[1,6]*C2
    RLjdqd2_32 = -s.dpt[1,6]*S2
    OMjdqd2_22 = qd[2]+qd[3]
    ORjdqd2_12 = RLjdqd2_32*qd[2]
    ORjdqd2_32 = -RLjdqd2_12*qd[2]
    Apqpjdqd2_12 = ORjdqd2_32*qd[2]
    Apqpjdqd2_32 = -ORjdqd2_12*qd[2]
    RLjdqd2_13 = ROjdqd2_72*s.dpt[3,7]
    RLjdqd2_33 = ROjdqd2_92*s.dpt[3,7]
    ORjdqd2_13 = OMjdqd2_22*RLjdqd2_33
    ORjdqd2_33 = -OMjdqd2_22*RLjdqd2_13
    Apqpjdqd2_13 = Apqpjdqd2_12+OMjdqd2_22*ORjdqd2_33
    Apqpjdqd2_33 = Apqpjdqd2_32-OMjdqd2_22*ORjdqd2_13
    jdqd1 = Apqpjdqd1_12-Apqpjdqd2_13
    jdqd3 = Apqpjdqd1_32-Apqpjdqd2_33
    Jdqd[1] = jdqd1
    Jdqd[2] = jdqd3

# Number of continuation lines = 0


