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
#	==> Function: F6 - Sensors Kinematics
#
#	==> Git hash: 0e4e6a608eeee06956095d2f2ad315abdb092777
#
##

from math import sin, cos, sqrt

def sensor(sens, s, isens):
  q = s.q
  qd = s.qd
  qdd = s.qdd

  dpt = s.dpt
 
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

 
# Sensor Kinematics


  if (isens == 1): 

    sens.P[1] = s.dpt[1,1]
    sens.P[2] = 0
    sens.P[3] = s.dpt[3,1]
    sens.R[1,1] = C1
    sens.R[1,3] = -S1
    sens.R[2,2] = (1.0)
    sens.R[3,1] = S1
    sens.R[3,3] = C1
    sens.V[1] = 0
    sens.V[2] = 0
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = qd[1]
    sens.OM[3] = 0
    sens.J[5,1] = (1.0)
    sens.A[1] = 0
    sens.A[2] = 0
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = qdd[1]
    sens.OMP[3] = 0

  if (isens == 2): 

    sens.P[1] = s.dpt[1,2]
    sens.P[2] = 0
    sens.P[3] = s.dpt[3,2]
    sens.R[1,1] = C2
    sens.R[1,3] = -S2
    sens.R[2,2] = (1.0)
    sens.R[3,1] = S2
    sens.R[3,3] = C2
    sens.V[1] = 0
    sens.V[2] = 0
    sens.V[3] = 0
    sens.OM[1] = 0
    sens.OM[2] = qd[2]
    sens.OM[3] = 0
    sens.J[5,2] = (1.0)
    sens.A[1] = 0
    sens.A[2] = 0
    sens.A[3] = 0
    sens.OMP[1] = 0
    sens.OMP[2] = qdd[2]
    sens.OMP[3] = 0

  if (isens == 3): 

    ROcp3_13 = C2*C3-S2*S3
    ROcp3_33 = -C2*S3-S2*C3
    ROcp3_73 = C2*S3+S2*C3
    ROcp3_93 = C2*C3-S2*S3
    RLcp3_12 = s.dpt[1,6]*C2
    RLcp3_32 = -s.dpt[1,6]*S2
    POcp3_12 = RLcp3_12+s.dpt[1,2]
    POcp3_32 = RLcp3_32+s.dpt[3,2]
    OMcp3_22 = qd[2]+qd[3]
    ORcp3_12 = RLcp3_32*qd[2]
    ORcp3_32 = -RLcp3_12*qd[2]
    OPcp3_22 = qdd[2]+qdd[3]
    ACcp3_12 = ORcp3_32*qd[2]+RLcp3_32*qdd[2]
    ACcp3_32 = -ORcp3_12*qd[2]-RLcp3_12*qdd[2]
    sens.P[1] = POcp3_12
    sens.P[2] = 0
    sens.P[3] = POcp3_32
    sens.R[1,1] = ROcp3_13
    sens.R[1,3] = ROcp3_33
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp3_73
    sens.R[3,3] = ROcp3_93
    sens.V[1] = ORcp3_12
    sens.V[2] = 0
    sens.V[3] = ORcp3_32
    sens.OM[1] = 0
    sens.OM[2] = OMcp3_22
    sens.OM[3] = 0
    sens.J[1,2] = RLcp3_32
    sens.J[3,2] = -RLcp3_12
    sens.J[5,2] = (1.0)
    sens.J[5,3] = (1.0)
    sens.A[1] = ACcp3_12
    sens.A[2] = 0
    sens.A[3] = ACcp3_32
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp3_22
    sens.OMP[3] = 0

  if (isens == 4): 

    ROcp4_13 = C2*C3-S2*S3
    ROcp4_33 = -C2*S3-S2*C3
    ROcp4_73 = C2*S3+S2*C3
    ROcp4_93 = C2*C3-S2*S3
    ROcp4_44 = ROcp4_73*S4
    ROcp4_64 = ROcp4_93*S4
    ROcp4_74 = ROcp4_73*C4
    ROcp4_94 = ROcp4_93*C4
    RLcp4_12 = s.dpt[1,6]*C2
    RLcp4_32 = -s.dpt[1,6]*S2
    POcp4_12 = RLcp4_12+s.dpt[1,2]
    POcp4_32 = RLcp4_32+s.dpt[3,2]
    OMcp4_22 = qd[2]+qd[3]
    ORcp4_12 = RLcp4_32*qd[2]
    ORcp4_32 = -RLcp4_12*qd[2]
    OPcp4_22 = qdd[2]+qdd[3]
    ACcp4_12 = ORcp4_32*qd[2]+RLcp4_32*qdd[2]
    ACcp4_32 = -ORcp4_12*qd[2]-RLcp4_12*qdd[2]
    RLcp4_13 = ROcp4_13*s.dpt[1,8]+ROcp4_73*s.dpt[3,8]
    RLcp4_33 = ROcp4_33*s.dpt[1,8]+ROcp4_93*s.dpt[3,8]
    POcp4_13 = POcp4_12+RLcp4_13
    POcp4_33 = POcp4_32+RLcp4_33
    JTcp4_13_1 = RLcp4_32+RLcp4_33
    JTcp4_33_1 = -RLcp4_12-RLcp4_13
    OMcp4_13 = ROcp4_13*qd[4]
    OMcp4_33 = ROcp4_33*qd[4]
    ORcp4_13 = OMcp4_22*RLcp4_33
    ORcp4_33 = -OMcp4_22*RLcp4_13
    VIcp4_13 = ORcp4_12+ORcp4_13
    VIcp4_33 = ORcp4_32+ORcp4_33
    OPcp4_13 = OMcp4_22*ROcp4_33*qd[4]+ROcp4_13*qdd[4]
    OPcp4_33 = -OMcp4_22*ROcp4_13*qd[4]+ROcp4_33*qdd[4]
    ACcp4_13 = ACcp4_12+OMcp4_22*ORcp4_33+OPcp4_22*RLcp4_33
    ACcp4_33 = ACcp4_32-OMcp4_22*ORcp4_13-OPcp4_22*RLcp4_13
    sens.P[1] = POcp4_13
    sens.P[2] = 0
    sens.P[3] = POcp4_33
    sens.R[1,1] = ROcp4_13
    sens.R[1,3] = ROcp4_33
    sens.R[2,1] = ROcp4_44
    sens.R[2,2] = C4
    sens.R[2,3] = ROcp4_64
    sens.R[3,1] = ROcp4_74
    sens.R[3,2] = -S4
    sens.R[3,3] = ROcp4_94
    sens.V[1] = VIcp4_13
    sens.V[2] = 0
    sens.V[3] = VIcp4_33
    sens.OM[1] = OMcp4_13
    sens.OM[2] = OMcp4_22
    sens.OM[3] = OMcp4_33
    sens.J[1,2] = JTcp4_13_1
    sens.J[1,3] = RLcp4_33
    sens.J[3,2] = JTcp4_33_1
    sens.J[3,3] = -RLcp4_13
    sens.J[4,4] = ROcp4_13
    sens.J[5,2] = (1.0)
    sens.J[5,3] = (1.0)
    sens.J[6,4] = ROcp4_33
    sens.A[1] = ACcp4_13
    sens.A[2] = 0
    sens.A[3] = ACcp4_33
    sens.OMP[1] = OPcp4_13
    sens.OMP[2] = OPcp4_22
    sens.OMP[3] = OPcp4_33

 


# Number of continuation lines = 0


