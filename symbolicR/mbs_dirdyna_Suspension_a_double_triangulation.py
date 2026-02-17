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
#	==> Function: F1 - Recursive Direct Dynamics of tree-like MBS
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos

def dirdyna(M, c, s, tsim):
    q = s.q
    qd = s.qd
 
# Trigonometric functions

    S1 = sin(q[1])
    C1 = cos(q[1])
    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
    S4 = sin(q[4])
    C4 = cos(q[4])
 
# Forward Kinematics

    AF31 = -s.g[3]*C1
    BS12 = -qd[2]*qd[2]
    AF12 = s.g[3]*S2
    AF32 = -s.g[3]*C2
    OM23 = qd[2]+qd[3]
    BS13 = -OM23*OM23
    BS93 = -OM23*OM23
    AF13 = -AF32*S3+C3*(AF12+BS12*s.dpt[1,6])
    AF33 = AF32*C3+S3*(AF12+BS12*s.dpt[1,6])
    AM13_2 = s.dpt[1,6]*S3
    AM33_2 = -s.dpt[1,6]*C3
 
# Backward Dynamics

    FA13 = -s.frc[1,3]+s.m[3]*(AF13+BS13*s.l[1,3])
    FA33 = -s.frc[3,3]+s.m[3]*(AF33+BS93*s.l[3,3])
    CF23 = -s.trq[2,3]+FA13*s.l[3,3]-FA33*s.l[1,3]
    FB13_2 = s.m[3]*(AM13_2+s.l[3,3])
    FB33_2 = s.m[3]*(AM33_2-s.l[1,3])
    CM23_2 = FB13_2*s.l[3,3]-FB33_2*s.l[1,3]
    FB13_3 = s.m[3]*s.l[3,3]
    FB33_3 = -s.m[3]*s.l[1,3]
    CM23_3 = FB13_3*s.l[3,3]-FB33_3*s.l[1,3]
    FA32 = -s.frc[3,2]+s.m[2]*AF32
    CF22 = -s.trq[2,2]+CF23-FA32*s.l[1,2]-s.dpt[1,6]*(-FA13*S3+FA33*C3)
    FB32_2 = -s.m[2]*s.l[1,2]
    CM22_2 = CM23_2-FB32_2*s.l[1,2]-s.dpt[1,6]*(-FB13_2*S3+FB33_2*C3)
    FA31 = -s.frc[3,1]+s.m[1]*AF31
    CF21 = -s.trq[2,1]-FA31*s.l[1,1]
    FB31_1 = -s.m[1]*s.l[1,1]
    CM21_1 = -FB31_1*s.l[1,1]
 
# Symbolic model output

    c[1] = CF21
    c[2] = CF22
    c[3] = CF23
    M[1,1] = CM21_1
    M[2,2] = CM22_2
    M[2,3] = CM23_2
    M[3,2] = CM23_2
    M[3,3] = CM23_3

# Number of continuation lines = 0


