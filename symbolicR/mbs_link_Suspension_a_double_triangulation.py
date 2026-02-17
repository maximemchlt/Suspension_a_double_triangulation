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
#	==> Generation Date: Tue Feb 17 18:10:01 2026
#	==> using automatic loading with extension .mbs 
#
#	==> Project name: Suspension_a_double_triangulation
#
#	==> Number of joints: 4
#
#	==> Function: F7 - Link Forces (1D)
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos, sqrt

def link(frc, trq, Flink, Z, Zd, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S2 = sin(q[2])
    C2 = cos(q[2])
 
# Augmented Joint Position Vectors

 
# Link anchor points Kinematics

    RLlnk2_12 = s.dpt[1,6]*C2+s.dpt[3,6]*S2
    RLlnk2_32 = -s.dpt[1,6]*S2+s.dpt[3,6]*C2
    POlnk2_12 = RLlnk2_12+s.dpt[1,2]
    ORlnk2_12 = qd[2]*RLlnk2_32
    ORlnk2_32 = -qd[2]*RLlnk2_12
    Plnk11 = POlnk2_12-s.dpt[1,4]
    Plnk31 = RLlnk2_32-s.dpt[3,4]
    PPlnk1 = Plnk11*Plnk11+Plnk31*Plnk31
    Z1 = sqrt(PPlnk1)
    e11 = Plnk11/Z1
    e31 = Plnk31/Z1
    Zd1 = ORlnk2_12*e11+ORlnk2_32*e31

# Link Forces 

    Flink1 = s.user_LinkForces(Z1,Zd1,s,tsim,1)
 
# Link Dynamics: forces projection on body-fixed frames

    fPlnk11 = Flink1*e11
    fPlnk31 = Flink1*e31
    trqlnk1_1_2 = fPlnk11*(s.dpt[3,4]-s.l[3,1])-fPlnk31*s.dpt[1,4]
    fSlnk11 = Flink1*(e11*C2-e31*S2)
    fSlnk31 = Flink1*(e11*S2+e31*C2)
    trqlnk2_1_2 = -fSlnk11*s.dpt[3,6]+fSlnk31*(s.dpt[1,6]-s.l[1,2])
 
# Symbolic model output

    frc[1,1] = s.frc[1,1]+fPlnk11
    frc[3,1] = s.frc[3,1]+fPlnk31
    trq[2,1] = s.trq[2,1]+trqlnk1_1_2
    frc[1,2] = s.frc[1,2]-fSlnk11
    frc[3,2] = s.frc[3,2]-fSlnk31
    trq[2,2] = s.trq[2,2]+trqlnk2_1_2
 
# Symbolic model output

    Z[1] = Z1
    Zd[1] = Zd1
    Flink[1] = Flink1

# Number of continuation lines = 0


