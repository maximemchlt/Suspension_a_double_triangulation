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

  S2 = sin(q[2])
  C2 = cos(q[2])
  S3 = sin(q[3])
  C3 = cos(q[3])
  S4 = sin(q[4])
  C4 = cos(q[4])
 
# Augmented Joint Position Vectors

 
# Sensor Kinematics


  if (isens == 1): 

    sens.P[1] = 0
    sens.P[2] = 0
    sens.P[3] = q[1]
    sens.R[1,1] = (1.0)
    sens.R[2,2] = (1.0)
    sens.R[3,3] = (1.0)
    sens.V[1] = 0
    sens.V[2] = 0
    sens.V[3] = qd[1]
    sens.OM[1] = 0
    sens.OM[2] = 0
    sens.OM[3] = 0
    sens.J[3,1] = (1.0)
    sens.A[1] = 0
    sens.A[2] = 0
    sens.A[3] = qdd[1]
    sens.OMP[1] = 0
    sens.OMP[2] = 0
    sens.OMP[3] = 0

  if (isens == 2): 

    sens.P[1] = s.dpt[1,2]
    sens.P[2] = 0
    sens.P[3] = q[1]
    sens.R[1,1] = C2
    sens.R[1,3] = -S2
    sens.R[2,2] = (1.0)
    sens.R[3,1] = S2
    sens.R[3,3] = C2
    sens.V[1] = 0
    sens.V[2] = 0
    sens.V[3] = qd[1]
    sens.OM[1] = 0
    sens.OM[2] = qd[2]
    sens.OM[3] = 0
    sens.J[3,1] = (1.0)
    sens.J[5,2] = (1.0)
    sens.A[1] = 0
    sens.A[2] = 0
    sens.A[3] = qdd[1]
    sens.OMP[1] = 0
    sens.OMP[2] = qdd[2]
    sens.OMP[3] = 0

  if (isens == 3): 

    POcp3_32 = q[1]+s.dpt[3,3]
    sens.P[1] = s.dpt[1,3]
    sens.P[2] = 0
    sens.P[3] = POcp3_32
    sens.R[1,1] = C3
    sens.R[1,3] = -S3
    sens.R[2,2] = (1.0)
    sens.R[3,1] = S3
    sens.R[3,3] = C3
    sens.V[1] = 0
    sens.V[2] = 0
    sens.V[3] = qd[1]
    sens.OM[1] = 0
    sens.OM[2] = qd[3]
    sens.OM[3] = 0
    sens.J[3,1] = (1.0)
    sens.J[5,3] = (1.0)
    sens.A[1] = 0
    sens.A[2] = 0
    sens.A[3] = qdd[1]
    sens.OMP[1] = 0
    sens.OMP[2] = qdd[3]
    sens.OMP[3] = 0

  if (isens == 4): 

    ROcp4_14 = C3*C4-S3*S4
    ROcp4_34 = -C3*S4-S3*C4
    ROcp4_74 = C3*S4+S3*C4
    ROcp4_94 = C3*C4-S3*S4
    POcp4_32 = q[1]+s.dpt[3,3]
    RLcp4_13 = s.dpt[1,7]*C3
    RLcp4_33 = -s.dpt[1,7]*S3
    POcp4_13 = RLcp4_13+s.dpt[1,3]
    POcp4_33 = POcp4_32+RLcp4_33
    OMcp4_23 = qd[3]+qd[4]
    ORcp4_13 = RLcp4_33*qd[3]
    ORcp4_33 = -RLcp4_13*qd[3]
    VIcp4_33 = ORcp4_33+qd[1]
    OPcp4_23 = qdd[3]+qdd[4]
    ACcp4_13 = ORcp4_33*qd[3]+RLcp4_33*qdd[3]
    ACcp4_33 = qdd[1]-ORcp4_13*qd[3]-RLcp4_13*qdd[3]
    sens.P[1] = POcp4_13
    sens.P[2] = 0
    sens.P[3] = POcp4_33
    sens.R[1,1] = ROcp4_14
    sens.R[1,3] = ROcp4_34
    sens.R[2,2] = (1.0)
    sens.R[3,1] = ROcp4_74
    sens.R[3,3] = ROcp4_94
    sens.V[1] = ORcp4_13
    sens.V[2] = 0
    sens.V[3] = VIcp4_33
    sens.OM[1] = 0
    sens.OM[2] = OMcp4_23
    sens.OM[3] = 0
    sens.J[1,3] = RLcp4_33
    sens.J[3,1] = (1.0)
    sens.J[3,3] = -RLcp4_13
    sens.J[5,3] = (1.0)
    sens.J[5,4] = (1.0)
    sens.A[1] = ACcp4_13
    sens.A[2] = 0
    sens.A[3] = ACcp4_33
    sens.OMP[1] = 0
    sens.OMP[2] = OPcp4_23
    sens.OMP[3] = 0

 


# Number of continuation lines = 0


