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
#	==> Function: F2 - Recursive Inverse Dynamics of tree-like MBS
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def invdyna(phi,s,tsim):
    Qq = phi  # compatibility with symbolic generation
    q = s.q
    qd = s.qd
    qdd = s.qdd
 
# Trigonometric functions

    S1 = sin(q[1])
    C1 = cos(q[1])
    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
    S4 = sin(q[4])
    C4 = cos(q[4])
 
# Augmented Joint Position Vectors

 
# Forward Kinematics

    ALPHA31 = -s.g[3]*C1
    BS12 = -qd[2]*qd[2]
    ALPHA12 = s.g[3]*S2
    ALPHA32 = -s.g[3]*C2
    OM23 = qd[2]+qd[3]
    OMp23 = qdd[2]+qdd[3]
    BS13 = -OM23*OM23
    BS93 = -OM23*OM23
    ALPHA13 = C3*(ALPHA12+BS12*s.dpt[1,6])-S3*(ALPHA32-qdd[2]*s.dpt[1,6])
    ALPHA33 = C3*(ALPHA32-qdd[2]*s.dpt[1,6])+S3*(ALPHA12+BS12*s.dpt[1,6])
 
# Backward Dynamics

    Fs13 = -s.frc[1,3]+s.m[3]*(ALPHA13+BS13*s.l[1,3]+OMp23*s.l[3,3])
    Fs33 = -s.frc[3,3]+s.m[3]*(ALPHA33+BS93*s.l[3,3]-OMp23*s.l[1,3])
    Cq23 = -s.trq[2,3]+Fs13*s.l[3,3]-Fs33*s.l[1,3]
    Fs32 = -s.frc[3,2]+s.m[2]*(ALPHA32-qdd[2]*s.l[1,2])
    Cq22 = -s.trq[2,2]+Cq23-Fs32*s.l[1,2]-s.dpt[1,6]*(-Fs13*S3+Fs33*C3)
    Fs31 = -s.frc[3,1]+s.m[1]*(ALPHA31-qdd[1]*s.l[1,1])
    Cq21 = -s.trq[2,1]-Fs31*s.l[1,1]
 
# Symbolic model output

    Qq[1] = Cq21
    Qq[2] = Cq22
    Qq[3] = Cq23

# Number of continuation lines = 0


