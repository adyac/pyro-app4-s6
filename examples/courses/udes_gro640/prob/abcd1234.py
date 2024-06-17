#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu May 14 23:19:16 2020

@author: alex
------------------------------------


Fichier d'amorce pour les livrables de la problématique GRO640'


"""

import numpy as np

from pyro.control  import robotcontrollers
from pyro.control.robotcontrollers import EndEffectorPD
from pyro.control.robotcontrollers import EndEffectorKinematicController


###################
# Part 1
###################

def dh2T( r , d , theta, alpha ):
    """

    Parameters
    ----------
    r     : float 1x1
    d     : float 1x1
    theta : float 1x1
    alpha : float 1x1
    
    4 paramètres de DH

    Returns
    -------
    T     : float 4x4 (numpy array)
            Matrice de transformation

    """
<<<<<<< Updated upstream
    
    T = np.zeros((4,4))
=======

    T = np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), r*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), r*np.sin(theta)],
        [0            ,               np.sin(alpha),                np.cos(alpha),                d],
        [0            ,                           0,                            0,                1]
    ])
>>>>>>> Stashed changes
    
    return T



def dhs2T( r , d , theta, alpha ):
    """

    Parameters
    ----------
    r     : float nx1
    d     : float nx1
    theta : float nx1
    alpha : float nx1
    
    Colonnes de paramètre de DH

    Returns
    -------
    WTT     : float 4x4 (numpy array)
              Matrice de transformation totale de l'outil

    """
<<<<<<< Updated upstream
    
=======
>>>>>>> Stashed changes
    WTT = np.zeros((4,4))
    
    ###################
    # Votre code ici
<<<<<<< Updated upstream
    ###################
=======
    dof = len(r) # nombre de joints du robot
    WTT = np.eye(4) # Matrice de transformation vide
   
    for i in range(dof):
        # itérer à travers ch joint.
        # Multiplier la matrice du nouveau joint avec la matrice du joint
        # précédent.
        WTT = np.dot(WTT, dh2T(r[i], d[i], theta[i], alpha[i]))
    return WTT
>>>>>>> Stashed changes
    


def f(q):
    """
    Parameters
    ----------
    q : float 6x1
        Joint space coordinates
    Returns
    -------
    r : float 3x1 
        Effector (x,y,z) position
    """
    r = np.zeros((3,1))
<<<<<<< Updated upstream
    
    ###################
    # Votre code ici
    ###################
    
=======

    # Paramètres du robot KUKA
    r_vect= np.array([0.147,0.155,0,0])
    d= np.array([0,0,0.135,0.217])
    theta= np.array([q[0], q[1], q[2]-np.pi/2, q[3]])
    alpha=np.array([np.pi/2, q[2], 0, q[4]])

    WTT = dhs2T(r_vect, d, theta, alpha)
    
    # paramètres position de la matrice de transformation
    r = WTT[:3, 3] # prend les 3 premiers elem de la 4e colonne de la matrice transf
>>>>>>> Stashed changes
    return r


###################
# Part 2
###################
    
class CustomPositionController( EndEffectorKinematicController ) :
    
    ############################
    def __init__(self, manipulator ):
        """ """
        
        EndEffectorKinematicController.__init__( self, manipulator, 1)
        
        ###################################################
        # Vos paramètres de loi de commande ici !!
        ###################################################
        
    
    #############################
    def c( self , y , r , t = 0 ):
        """ 
        Feedback law: u = c(y,r,t)
        
        INPUTS
        y = q   : sensor signal vector  = joint angular positions      dof x 1
        r = r_d : reference signal vector  = desired effector position   e x 1
        t       : time                                                   1 x 1
        
        OUPUTS
        u = dq  : control inputs vector =  joint velocities             dof x 1
        
        """

        f_z_desired = 200  # Newtons
        f_e = np.array([0, 0, f_z_desired]) #forces en x y et z

        # Feedback from sensors
        x = y

        # Modele du robot
        r = self.robot_model.forward_kinematic_effector(q)
        J = self.robot_model.J(q)      # Jacobian matrix
        g = self.robot_model.g(q)      # Gravity vector
        H = self.robot_model.H(q)      # Inertia matrix
        C = self.robot_model.C(q, dq)  # Coriolis matrix

        # Impedance control parameters
        Kp = np.diag([1000, 1000, 0])  
        Kd = np.diag([100, 100, 0]) 

        # Jacobian computation
        J = self.J(q)
        
        # Ref
        r_desired   = r
        r_actual    = self.fwd_kin(q)
        
        # Error
        e  = r_desired - r_actual
        
        ################
        dq = np.zeros( self.m )  # place-holder de bonne dimension
        
        ##################################
        # Votre loi de commande ici !!!
        ##################################

        
        return dq
    
    
###################
# Part 3
###################
        

        
class CustomDrillingController( robotcontrollers.RobotController ) :
    """ 

    """
    
    ############################
    def __init__(self, robot_model ):
        """ """
        
        super().__init__( dof = 3 )
        
        self.robot_model = robot_model
        
        # Label
        self.name = 'Custom Drilling Controller'
        
        
    #############################
    def c( self , y , r , t = 0 ):
        """ 
        Feedback static computation u = c(y,r,t)
        
        INPUTS
        y  : sensor signal vector     p x 1
        r  : reference signal vector  k x 1
        t  : time                     1 x 1
        
        OUPUTS
        u  : control inputs vector    m x 1
        
        """
        
        # Ref
        f_e = r
        
        # Feedback from sensors
        x = y
        [ q , dq ] = self.x2q( x )
        
        # Robot model
        r = self.robot_model.forward_kinematic_effector( q ) # End-effector actual position
        J = self.robot_model.J( q )      # Jacobian matrix
        g = self.robot_model.g( q )      # Gravity vector
        H = self.robot_model.H( q )      # Inertia matrix
        C = self.robot_model.C( q , dq ) # Coriolis matrix
            
        ##################################
        # Votre loi de commande ici !!!
        ##################################
        
        u = np.zeros(self.m)  # place-holder de bonne dimension
        
        return u
        
    
###################
# Part 4
###################
        
    
def goal2r( r_0 , r_f , t_f ):
    """
    
    Parameters
    ----------
    r_0 : numpy array float 3 x 1
        effector initial position
    r_f : numpy array float 3 x 1
        effector final position
    t_f : float
        time 

    Returns
    -------
    r   : numpy array float 3 x l
    dr  : numpy array float 3 x l
    ddr : numpy array float 3 x l

    """
    # Time discretization
    l = 1000 # nb of time steps
    t_vect = np.linspace(0, t_f, l)

    # Number of DoF for the effector only
    m = 3
    
    r = np.zeros((m,l))
    dr = np.zeros((m,l))
    ddr = np.zeros((m,l))

    # Pour chaque DOF
    for i in range(m):

        a0 = r_0[i] # Pos initiale
        a1 = 0      # vit initiale

        # Equation 1: r(t) = a0 + a1*t +a2 * t^2 + a3 * t^3
        # ou a0 = r0, donc
        # a2 *t^2 + a3*t^3 = rf - r0

        # Equation 2: r(t) = a0 + a1*t +a2 * t^2 + a3 * t^3
        # dr(t) = a1 + 2*a2*t + 3 *a3 * t^2 
        # 0 = 0 + 2*a2*t + 3a3*tf^2 
        # ...
        # a2 = -3/2 * a3*t
        # Substitution 1 dans 2...
        # a3 = 2(rf-r0)/tf^3
        # a2 = 3(rf-r0)/tf^2

        a2 = 3 * (r_f[i] - r_0[i]) / t_f**2 # 
        a3 = -2 * (r_f[i] - r_0[i]) / t_f**3

        # les matrices représentant la trajectoire temporelle des pos, vit, accel cartésiennes de l'effecteur
        r[i, :] = a0 + a1 * t_vect + a2 * t_vect**2 + a3 * t_vect**3
        dr[i, :] = a1 + 2 * a2 * t_vect + 3 * a3 * t_vect**2
        ddr[i, :] = 2 * a2 + 6 * a3 * t_vect

    return r, dr, ddr
    
    


def r2q( r, dr, ddr , manipulator ):
    """

    Parameters
    ----------
    r   : numpy array float 3 x l
    dr  : numpy array float 3 x l
    ddr : numpy array float 3 x l
    
    manipulator : pyro object 

    Returns
    -------
    q   : numpy array float 3 x l
    dq  : numpy array float 3 x l
    ddq : numpy array float 3 x l

    """
    # Time discretization
    l = r.shape[1]
    # Number of DoF
    n = 3
    
    # Output dimensions
    q = np.zeros((n,l))
    dq = np.zeros((n,l))
    ddq = np.zeros((n,l))
    
    # Pour ch time step
    for i in range(l):
        # On veut jacobienne inv de chaque configuration de joint
        J_inv = np.linalg.pinv(manipulator.J(q[:, i]))

        # angles des joint q, données selon positions r cartésiennes de l'outil
        q[:, i] = manipulator.inverse_kinematics(r[:, i])

        # dq = J^-1 * dr pour passer de cartésien à joint
        dq[:, i] = J_inv @ dr[:, i]

        # ddq = J^-1 (ddr - dJ/dt * dq)
        ddq[:, i] = J_inv @ ddr[:, i] - manipulator.J_dot(q[:, i], dq[:, i]) @ dq[:, i]

    return q, dq, ddq
    


def q2torque( q, dq, ddq , manipulator ):
    """

    Parameters
    ----------
    q   : numpy array float 3 x l
    dq  : numpy array float 3 x l
    ddq : numpy array float 3 x l
    
    manipulator : pyro object 

    Returns
    -------
    tau   : numpy array float 3 x l

    """
    # Time discretization
    l = q.shape[1]
    
    # Number of DoF
    n = 3
    
    # Output dimensions
    tau = np.zeros((n,l))

    for i in range(l):

        H = manipulator.H(q[:, i])  # matrice Inertie
        C = manipulator.C(q[:, i], dq[:, i])  # Coriolis matrix
        g = manipulator.g(q[:, i])  # Gravity vector

        # Calculer torques à partir du modèle dynamique
        # torque = H*ddq + c* dq + gravité
        tau[:, i] = H @ ddq[:, i] + C @ dq[:, i] + g

    return tau
    
    
    return tau