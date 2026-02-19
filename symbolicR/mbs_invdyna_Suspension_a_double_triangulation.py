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

    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
 
# Augmented Joint Position Vectors

 
# Forward Kinematics

    ALPHA31 = qdd[1]-s.g[3]
    BS12 = -qd[2]*qd[2]
    ALPHA12 = -ALPHA31*S2
    ALPHA32 = ALPHA31*C2
    BS13 = -qd[3]*qd[3]
    ALPHA13 = -ALPHA31*S3
    ALPHA33 = ALPHA31*C3
    OM24 = qd[3]+qd[4]
    OMp24 = qdd[3]+qdd[4]
    BS14 = -OM24*OM24
    BS94 = -OM24*OM24
    ALPHA14 = C4*(ALPHA13+BS13*s.dpt[1,7])-S4*(ALPHA33-qdd[3]*s.dpt[1,7])
    ALPHA34 = C4*(ALPHA33-qdd[3]*s.dpt[1,7])+S4*(ALPHA13+BS13*s.dpt[1,7])
 
# Backward Dynamics

    Fs14 = -s.frc[1,4]+s.m[4]*(ALPHA14+BS14*s.l[1,4]+OMp24*s.l[3,4])
    Fs34 = -s.frc[3,4]+s.m[4]*(ALPHA34+BS94*s.l[3,4]-OMp24*s.l[1,4])
    Cq24 = -s.trq[2,4]+s.In[5,4]*OMp24+Fs14*s.l[3,4]-Fs34*s.l[1,4]
    Fs13 = -s.frc[1,3]+s.m[3]*(ALPHA13+BS13*s.l[1,3])
    Fs33 = -s.frc[3,3]+s.m[3]*(ALPHA33-qdd[3]*s.l[1,3])
    Fq13 = Fs13+Fs14*C4+Fs34*S4
    Fq33 = Fs33-Fs14*S4+Fs34*C4
    Cq23 = -s.trq[2,3]+Cq24+qdd[3]*s.In[5,3]-Fs33*s.l[1,3]-s.dpt[1,7]*(-Fs14*S4+Fs34*C4)
    Fs12 = -s.frc[1,2]+s.m[2]*(ALPHA12+BS12*s.l[1,2])
    Fs32 = -s.frc[3,2]+s.m[2]*(ALPHA32-qdd[2]*s.l[1,2])
    Cq22 = -s.trq[2,2]+qdd[2]*s.In[5,2]-Fs32*s.l[1,2]
    Fs31 = -s.frc[3,1]+s.m[1]*ALPHA31
    Fq31 = Fs31-Fq13*S3+Fq33*C3-Fs12*S2+Fs32*C2
 
# Symbolic model output

    Qq[1] = Fq31
    Qq[2] = Cq22
    Qq[3] = Cq23
    Qq[4] = Cq24

# Number of continuation lines = 0


