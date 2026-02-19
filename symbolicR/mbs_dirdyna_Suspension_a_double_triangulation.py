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

    S2 = sin(q[2])
    C2 = cos(q[2])
    S3 = sin(q[3])
    C3 = cos(q[3])
    S4 = sin(q[4])
    C4 = cos(q[4])
    S5 = sin(q[5])
    C5 = cos(q[5])
 
# Forward Kinematics

    BS12 = -qd[2]*qd[2]
    AF12 = s.g[3]*S2
    AF32 = -s.g[3]*C2
    BS13 = -qd[3]*qd[3]
    AF13 = s.g[3]*S3
    AF33 = -s.g[3]*C3
    OM24 = qd[3]+qd[4]
    BS14 = -OM24*OM24
    BS94 = -OM24*OM24
    AF14 = -AF33*S4+C4*(AF13+BS13*s.dpt[1,7])
    AF34 = AF33*C4+S4*(AF13+BS13*s.dpt[1,7])
    AM14_1 = -C3*S4-S3*C4
    AM34_1 = C3*C4-S3*S4
    AM14_3 = s.dpt[1,7]*S4
    AM34_3 = -s.dpt[1,7]*C4
 
# Backward Dynamics

    FA14 = -s.frc[1,4]+s.m[4]*(AF14+BS14*s.l[1,4])
    FA34 = -s.frc[3,4]+s.m[4]*(AF34+BS94*s.l[3,4])
    CF24 = -s.trq[2,4]+FA14*s.l[3,4]-FA34*s.l[1,4]
    FB14_1 = s.m[4]*AM14_1
    FB34_1 = s.m[4]*AM34_1
    CM24_1 = FB14_1*s.l[3,4]-FB34_1*s.l[1,4]
    FB14_3 = s.m[4]*(AM14_3+s.l[3,4])
    FB34_3 = s.m[4]*(AM34_3-s.l[1,4])
    CM24_3 = s.In[5,4]+FB14_3*s.l[3,4]-FB34_3*s.l[1,4]
    FB14_4 = s.m[4]*s.l[3,4]
    FB34_4 = -s.m[4]*s.l[1,4]
    CM24_4 = s.In[5,4]+FB14_4*s.l[3,4]-FB34_4*s.l[1,4]
    FA13 = -s.frc[1,3]+s.m[3]*(AF13+BS13*s.l[1,3])
    FA33 = -s.frc[3,3]+s.m[3]*AF33
    FF13 = FA13+FA14*C4+FA34*S4
    FF33 = FA33-FA14*S4+FA34*C4
    CF23 = -s.trq[2,3]+CF24-FA33*s.l[1,3]-s.dpt[1,7]*(-FA14*S4+FA34*C4)
    FB13_1 = -s.m[3]*S3
    FB33_1 = s.m[3]*C3
    FM13_1 = FB13_1+FB14_1*C4+FB34_1*S4
    FM33_1 = FB33_1-FB14_1*S4+FB34_1*C4
    CM23_1 = CM24_1-FB33_1*s.l[1,3]-s.dpt[1,7]*(-FB14_1*S4+FB34_1*C4)
    FB33_3 = -s.m[3]*s.l[1,3]
    CM23_3 = s.In[5,3]+CM24_3-FB33_3*s.l[1,3]-s.dpt[1,7]*(-FB14_3*S4+FB34_3*C4)
    FA12 = -s.frc[1,2]+s.m[2]*(AF12+BS12*s.l[1,2])
    FA32 = -s.frc[3,2]+s.m[2]*AF32
    CF22 = -s.trq[2,2]-FA32*s.l[1,2]
    FB12_1 = -s.m[2]*S2
    FB32_1 = s.m[2]*C2
    CM22_1 = -FB32_1*s.l[1,2]
    FB32_2 = -s.m[2]*s.l[1,2]
    CM22_2 = s.In[5,2]-FB32_2*s.l[1,2]
    FA31 = -s.frc[3,1]-s.m[1]*s.g[3]
    FF31 = FA31-FA12*S2+FA32*C2-FF13*S3+FF33*C3
    FM31_1 = s.m[1]-FB12_1*S2+FB32_1*C2-FM13_1*S3+FM33_1*C3
 
# Symbolic model output

    c[1] = FF31
    c[2] = CF22
    c[3] = CF23
    c[4] = CF24
    M[1,1] = FM31_1
    M[1,2] = CM22_1
    M[1,3] = CM23_1
    M[1,4] = CM24_1
    M[2,1] = CM22_1
    M[2,2] = CM22_2
    M[3,1] = CM23_1
    M[3,3] = CM23_3
    M[3,4] = CM24_3
    M[4,1] = CM24_1
    M[4,3] = CM24_3
    M[4,4] = CM24_4

# Number of continuation lines = 0


