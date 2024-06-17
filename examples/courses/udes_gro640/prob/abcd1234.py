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
 
    T = np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), r*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), r*np.sin(theta)],
        [0            ,               np.sin(alpha),                np.cos(alpha),                d],
        [0            ,                           0,                            0,                1]
    ])
   
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
    WTT = np.zeros((4,4))
   
    ###################
    # Votre code ici
    dof = len(r) # nombre de joints du robot
    WTT = np.eye(4) # Matrice de transformation vide
   
    for i in range(dof):
        # itérer à travers ch joint.
        # Multiplier la matrice du nouveau joint avec la matrice du joint
        # précédent.
        WTT = np.dot(WTT, dh2T(r[i], d[i], theta[i], alpha[i]))
    return WTT
   
 
 
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
 
    # Paramètres du robot KUKA
    r_vect= np.array([30, 155 ,135 ,0,0])
    d= np.array([147,0,0,0,217])
    theta= np.array([q[0], q[1]+np.pi/2, q[2], -q[3]+np.pi/2, q[4]])
    alpha=np.array([-np.pi/2, 0, 0, np.pi/2,0])
 
    WTT = dhs2T(r_vect, d, theta, alpha)
   
    # paramètres position de la matrice de transformation
    r = WTT[:3, 3] # prend les 3 premiers elem de la 4e colonne de la matrice transf
    return r

###################
# Part 2
###################
    
class CustomPositionController( EndEffectorKinematicController ) :
    
    ############################
    def __init__(self, manipulator ):
        EndEffectorKinematicController.__init__( self, manipulator, 1)

        self.gain_unitaire = 0.5
        self.gain = np.array([4, 4]) # gain en fonction de self.e -> 2 degre lib effecteur
        
        self.gain_nul_space = np.array([0.3, 0.3, 0.3]) # essai erreur
        self.pos_joint_desire = np.array([0, 0.5, 0])
        #self.pos_joint_desire = np.array([0, 0.7, 0])
        #self.gain = np.diag([10, 1]) # gain matrice NxN ; N = nombre DDL effecteur
        # self.lamda = 1 # valeur propre 
        
    
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
        # Feedback from sensors
        q = y

        # Jacobian computation
        J = self.J( q )
        
        # Ref
        r_desired   = r
        r_actual    = self.fwd_kin( q )

        # Error
        e  = r_desired - r_actual

        ################
        # dq = np.zeros( self.m )  # place-holder de bonne dimension  # singularite q3 est a 180

        JtJ = np.dot(J.transpose(), J) + self.gain_unitaire**2 * np.identity(self.dof)
        J_pseudo = np.dot(np.linalg.inv(JtJ), J.transpose()) 
        Mat_espace_null = (np.identity(self.dof) - J_pseudo @ J)

        erreur_pos_joint = self.pos_joint_desire - q
        vitesse_null =  self.gain_nul_space * erreur_pos_joint

        vitesse_effecteur = self.gain * e

        #eigenvalues = np.linalg.eigvals(JtJ)
        #min_eigenvalue = np.min(eigenvalues[np.nonzero(eigenvalues)])
        #lamda_reel = min_eigenvalue + 0.01 
        #print(lamda_reel) 

        dq = J_pseudo @ vitesse_effecteur + Mat_espace_null @ vitesse_null
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
        self.state = 1
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
        
        # Feedback from sensors
        x = y
        [ q , dq ] = self.x2q( x )
        
        # Robot model
        r = self.robot_model.forward_kinematic_effector( q ) # End-effector actual position
        J = self.robot_model.J( q )      # Jacobian matrix
        J_T = J.transpose()
        g = self.robot_model.g( q )      # Gravity vector
        H = self.robot_model.H( q )      # Inertia matrix
        C = self.robot_model.C( q , dq ) # Coriolis matrix
        
            
        ##################################
        # Votre loi de commande ici !!!
        ##################################

        kp = np.diag([100, 100, 100])  # Matrice diagonale pour les gains proportionnels
        kd = np.diag([50, 50, 50])     # Matrice diagonale pour les gains dérivés

        if self.state == 1: # État 1 : alignement avec le pré-trou
            pos_des = np.array([0.25, 0.25, 0.40])  # Position désirée : (25 cm, 25 cm, 45 cm) au-dessus du pré-trou
            force = np.array([0, 0, 0])           # Pas de force de perçage appliquée
            if np.isclose(pos_des, r, atol=0.001).all():  # Vérifie si la position actuelle est proche de la position désirée avec une tolérance de 0.001
                self.state = 2  # Passer à l'état 2 pour commencer le perçage

        if self.state == 2: # État 2 : perçage
            pos_des = np.array([0.25, 0.25, 0.20]) # Position désirée : (25 cm, 25 cm, 20 cm) dans le trou
            force = np.array([0, 0, -200])       # Force de perçage appliquée vers le bas
            if np.isclose(pos_des, r, atol=0.001).all(): # Vérifie si la position actuelle est proche de la position désirée avec une tolérance de 0.001
                self.state = 3  # Passer à l'état 3 pour sortir du trou

        if self.state == 3: # État 3 : sortir du trou
            pos_des = np.array([0.25, 0.25, 0.40]) # Position désirée : (25 cm, 25 cm, 45 cm) au-dessus du trou
            force = np.array([0, 0, 0])          # Pas de force de perçage appliquée
            if np.isclose(pos_des, r, atol=0.001).all(): # Vérifie si la position actuelle est proche de la position désirée avec une tolérance de 0.001
                self.state = 1  # Revenir à l'état 1 pour un nouveau cycle

        u = J_T @ (kp @ (pos_des - r) + kd @ (-J @ dq)) + J_T @ force + g # Loi de contrôle (u)

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
   
    def inverse_kinematics(r, link_lengths):
        # r is a 3D Cartesian position vector [x, y, z]
        x, y, z = r
        L1, L2, L3 = link_lengths  # Lengths of the links
       
        # Calculate joint angles q for a 3-DOF manipulator
        q1 = np.arctan2(y, x)
       
        # Project onto the xy-plane for remaining calculations
        r_xy = np.sqrt(x**2 + y**2)
       
        q3 = np.arctan2(z, r_xy - L1)
       
        # Calculate q2 using the Law of Cosines
        c2 = (r_xy**2 + (z - L1)**2 - L2**2 - L3**2) / (2 * L2 * L3)
        s2 = np.sqrt(1 - c2**2)
        q2 = np.arctan2(s2, c2)
       
        return np.array([q1, q2, q3])
 
    # Example usage in your r2q function
    link_lengths = np.array([0.3, 0.525, 0.375])  # l1=0.3, l2=0.525, l3=0.375 from gro640
    

    # Pour ch time step
    for i in range(l):
        # On veut jacobienne inv de chaque configuration de joint
        J_inv = np.linalg.pinv(manipulator.J(q[:, i]))
 
        # angles des joint q, données selon positions r cartésiennes de l'outil
        q[:, i] = inverse_kinematics(r[:, i], link_lengths)
 
        # dq = J^-1 * dr pour passer de cartésien à joint
        dq[:, i] = J_inv @ dr[:, i]
 
        # ddq = J^-1 (ddr - dJ/dt * dq)
        ddq[:, i] = J_inv @ ddr[:, i] - manipulator.J(q[:, i]) @ dq[:, i]
 
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
