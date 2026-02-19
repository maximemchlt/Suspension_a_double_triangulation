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
#	==> Generation Date: Thu Feb 19 16:22:13 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Suspension_a_double_triangulation
#
#	==> Number of joints: 5
#
#	==> Function: F8 - Constraints and Constraints Jacobian(h, J)
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def cons_hJ(h, Jac, s):
    q = s.q
 
# Trigonometric functions

    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
    S4 = sin(q[4])
    C4 = cos(q[4])
 
# Augmented Joint Position Vectors

 
# Constraints and Constraints Jacobian

    RLlp1_12 = s.dpt[1,5]*C2
    RLlp1_32 = -s.dpt[1,5]*S2
    POlp1_12 = RLlp1_12+s.dpt[1,2]
    ROlp2_72 = C3*S4+S3*C4
    ROlp2_92 = C3*C4-S3*S4
    RLlp2_12 = s.dpt[1,7]*C3
    RLlp2_32 = -s.dpt[1,7]*S3
    POlp2_12 = RLlp2_12+s.dpt[1,3]
    POlp2_32 = RLlp2_32+s.dpt[3,3]
    RLlp2_13 = ROlp2_72*s.dpt[3,9]
    RLlp2_33 = ROlp2_92*s.dpt[3,9]
    POlp2_13 = POlp2_12+RLlp2_13
    POlp2_33 = POlp2_32+RLlp2_33
    JTlp2_13_1 = RLlp2_32+RLlp2_33
    JTlp2_33_1 = -RLlp2_12-RLlp2_13
    h_1 = POlp1_12-POlp2_13
    h_3 = -POlp2_33+RLlp1_32
    h[1] = h_1
    h[2] = h_3
    Jac[1,1] = 0
    Jac[1,2] = RLlp1_32
    Jac[1,3] = -JTlp2_13_1
    Jac[1,4] = -RLlp2_33
    Jac[1,5] = 0
    Jac[2,1] = 0
    Jac[2,2] = -RLlp1_12
    Jac[2,3] = -JTlp2_33_1
    Jac[2,4] = RLlp2_13
    Jac[2,5] = 0

# Number of continuation lines = 0


