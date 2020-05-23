#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri May  1 19:51:49 2020

@author: alex
"""

##############################################################################
import numpy as np
##############################################################################
from pyro.dynamic.manipulator import SpeedControlledManipulator
from pyro.dynamic.manipulator import ThreeLinkManipulator3D
##############################################################################

########################################
# Model de robot cinématique seulement
########################################

class LaserRobot( SpeedControlledManipulator ):
    """ 
    Robot
    """
    ###########################
    def __init__(self):
        """ """
        
        # Dimensions
        dof = 3
        e   = 2
               
        # initialize standard params
        SpeedControlledManipulator.__init__( self, dof, e)
        
        # Name
        self.name = 'Robot planaire de gravure laser'
        
        l1  = 1.2
        l2  = 0.5
        l3  = 0.5
        
        self.l = np.array([l1,l2,l3])
        
        
    ##############################
    def trig(self, q ):
        """ 
        Compute cos and sin usefull in other computation 
        ------------------------------------------------
        """
        
        c1  = np.cos( q[0] )
        s1  = np.sin( q[0] )
        c2  = np.cos( q[1] )
        s2  = np.sin( q[1] )
        c3  = np.cos( q[2] )
        s3  = np.sin( q[2] )
        
        cos_rel = np.array( [ c1 , c2 , c3 ])
        sin_rel = np.array( [ s1 , s2 , s3 ])
        
        c12    = np.cos( q[0] + q[1] )
        s12    = np.sin( q[0] + q[1] )
        c123   = np.cos( q[0] + q[1] + q[2])
        s123   = np.sin( q[0] + q[1] + q[2])
        
        cos_abs = np.array( [ c1 , c12 , c123 ])
        sin_abs = np.array( [ s1 , s12 , s123 ])
        
        return [cos_rel,sin_rel,cos_abs,sin_abs]
    
    
    ##############################
    def forward_kinematic_effector(self, q ):
        """ """
        
        [cos_rel,sin_rel,cos_abs,sin_abs] = self.trig( q )
        
        x = (self.l * sin_abs).sum()
        y = (self.l * cos_abs).sum()
        
        r = np.array([x,y])
        
        return r
    
    
    ##############################
    def J(self, q ):
        """ """
        
        [cos_rel,sin_rel,cos_abs,sin_abs] = self.trig( q )
        
        J = np.zeros( ( self.e  , self.dof ) ) # Place holder
        
        J[0,0] = self.l[2] * cos_abs[2] + self.l[1] * cos_abs[1]  + self.l[0] * cos_abs[0] 
        J[0,1] = self.l[2] * cos_abs[2] + self.l[1] * cos_abs[1]             
        J[0,2] = self.l[2] * cos_abs[2]           

        
        J[1,0] = - self.l[2] * sin_abs[2] - self.l[1] * sin_abs[1]  - self.l[0] * sin_abs[0] 
        J[1,1] = - self.l[2] * sin_abs[2] - self.l[1] * sin_abs[1]             
        J[1,2] = - self.l[2] * sin_abs[2]           
        
        return J
    
        
    ###########################################################################
    # Graphical output
    ###########################################################################
    
    ###########################################################################
    def forward_kinematic_domain(self, q ):
        """ 
        """
        l = 3
        
        domain  = [ (-l,l) , (-l,l) , (-l,l) ]#  
                
        return domain
    
    ###########################################################################
    def forward_kinematic_lines(self, q ):
        """ 
        Compute points p = [x;y;z] positions given config q 
        ----------------------------------------------------
        - points of interest for ploting
        
        Outpus:
        lines_pts = [] : a list of array (n_pts x 3) for each lines
        
        """
        
        lines_pts = [] # list of array (n_pts x 3) for each lines
        
        ###############################
        # ground line
        ###############################
        
        pts      = np.zeros(( 2 , 3 ))
        pts[0,:] = np.array([-10,0,0])
        pts[1,:] = np.array([+10,0,0])
        
        lines_pts.append( pts )
        
        ###########################
        # robot kinematic
        ###########################
        
        pts      = np.zeros(( 4 , 3 ))
        pts[0,:] = np.array([0,0,0])
        
        [cos_rel,sin_rel,cos_abs,sin_abs] = self.trig( q )
        
        pts[1,0] = self.l[0] * sin_abs[0]
        pts[2,0] = self.l[1] * sin_abs[1] + pts[1,0]
        pts[3,0] = self.l[2] * sin_abs[2] + pts[2,0]

        
        pts[1,1] = self.l[0] * cos_abs[0]
        pts[2,1] = self.l[1] * cos_abs[1] + pts[1,1]
        pts[3,1] = self.l[2] * cos_abs[2] + pts[2,1]

        
        lines_pts.append( pts )
        
        ###########################
        # Drill
        ###########################
        
        #pts      = np.zeros(( 2 , 3 ))
        
        #pts[0,:] = pts[3,1]
        
        #lines_pts.append( pts )
                
        return lines_pts
    



########################################
# Model de robot dynamiques
########################################

    
class DrillingRobot( ThreeLinkManipulator3D ):
    """ 
    """
    
    ############################
    def __init__(self):
        """ """
               
        # initialize standard params
        ThreeLinkManipulator3D.__init__(self)
        
        # Name
        self.name = 'Drilling Robot'
        
        # kinematic
        self.l1  = 0.3
        self.l2  = 0.525 
        self.l3  = 0.375
        
        
        # dynamic
        self.I1  = 0.66125
        self.m2  = 1.589
        self.m3  = 0.545
        
        self.gravity = 9.81
        
        
    ##############################
    def trig(self, q ):
        """ 
        Compute cos and sin usefull in other computation 
        ------------------------------------------------
        """
        
        c1  = np.cos( q[0] )
        s1  = np.sin( q[0] )
        c2  = np.cos( q[1] )
        s2  = np.sin( q[1] )
        c3  = np.cos( q[2] )
        s3  = np.sin( q[2] )
        c12 = np.cos( q[0] + q[1] )
        s12 = np.sin( q[0] + q[1] )
        c23 = np.cos( q[2] + q[1] )
        s23 = np.sin( q[2] + q[1] )
        
        return [c1,s1,c2,s2,c3,s3,c12,s12,c23,s23]
    
    
    ##############################
    def forward_kinematic_effector(self, q ):
        """ """
        
        [c1,s1,c2,s2,c3,s3,c12,s12,c23,s23] = self.trig( q )
        
        lz1     = self.l1
        lx2     = self.l2
        lx3     = self.l3
        
        # End-effector kinmatic

        x3 = c1*(lx3*c23 + lx2*c2)
        y3 = s1*(lx3*c23 + lx2*c2)
        z3 = lz1 + lx3*s23 + lx2*s2
                
        r = np.array([x3, y3, z3])
        
        return r
    
    
    ##############################
    def J(self, q ):
        """ """
        
        [c1,s1,c2,s2,c3,s3,c12,s12,c23,s23] = self.trig( q )
        
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        
        sin=np.sin
        cos=np.cos
        
        lx2     = self.l2
        lx3     = self.l3
        
        J = np.zeros((3,3))
        
        J[0,0] =  -sin(q1)*(lx3*cos(q2 + q3) + lx2*cos(q2))
        J[0,1] =  -cos(q1)*(lx3*sin(q2 + q3) + lx2*sin(q2))
        J[0,2] =  -lx3*sin(q2 + q3)*cos(q1)
        
        J[1,0] =   cos(q1)*(lx3*cos(q2 + q3) + lx2*cos(q2))
        J[1,1] =  -sin(q1)*(lx3*sin(q2 + q3) + lx2*sin(q2))        
        J[1,2] =  -lx3*sin(q2 + q3)*sin(q1)
        
        J[2,0] =  0
        J[2,1] =  lx3*cos(q2 + q3) + lx2*cos(q2)
        J[2,2] =  lx3*cos(q2 + q3)
        
        return J
    
    
    ###########################################################################
    def H(self, q ):
        """ 
        Inertia matrix 
        ----------------------------------
        dim( H ) = ( dof , dof )
        
        such that --> Kinetic Energy = 0.5 * dq**T * H(q) * dq
        
        """  
        
        [c1,s1,c2,s2,c3,s3,c12,s12,c23,s23] = self.trig( q )
        
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        
        sin=np.sin
        cos=np.cos
        
        lx2     = self.l2
        lx3     = self.l3
        
        I1      = self.I1
        m2      = self.m2
        m3      = self.m3
        
        H = np.zeros((3,3))
        
        H[0,0] = I1 + (m3*(2*(lx2*cos(q2)*sin(q1) + lx3*cos(q2)*cos(q3)*sin(q1) - lx3*sin(q1)*sin(q2)*sin(q3))**2 + 2*(lx2*cos(q1)*cos(q2) + lx3*cos(q1)*cos(q2)*cos(q3) - lx3*cos(q1)*sin(q2)*sin(q3))**2))/2 + lx2**2*m2*cos(q2)**2
        H[1,0] = 0        
        H[2,0] = 0
        
        H[0,1] = H[1,0]
        H[1,1] = (m3*(2*lx2**2 + 4*cos(q3)*lx2*lx3 + 2*lx3**2))/2 + lx2**2*m2
        H[2,1] = m3*(lx3**2 + lx2*cos(q3)*lx3)
        
        H[0,2] = H[2,0]
        H[1,2] = H[2,1]
        H[2,2] = lx3**2*m3        
        
        return H
    
    
    ###########################################################################
    def C(self, q , dq ):
        """ 
         Corriolis and Centrifugal Matrix 
        ------------------------------------
        dim( C ) = ( dof , dof )
        
        such that --> d H / dt =  C + C**T
        
        
        """ 
        
        [c1,s1,c2,s2,c3,s3,c12,s12,c23,s23] = self.trig( q )
        
        q2 = q[1]
        q3 = q[2]
        
        sin=np.sin
        
        q1d     = dq[0]
        q2d     = dq[1]
        q3d     = dq[2]
        
        lx2     = self.l2
        lx3     = self.l3
        
        m2      = self.m2
        m3      = self.m3
        
        C = np.zeros((3,3))
        
        C[0,0] = 0
        C[0,1] = -q1d*(lx3**2*m3*sin(2*q2 + 2*q3) + lx2**2*m2*sin(2*q2) + lx2**2*m3*sin(2*q2) + 2*lx2*lx3*m3*sin(2*q2 + q3))
        C[0,2] = -lx3*m3*q1d*(lx3*sin(2*q2 + 2*q3) + lx2*sin(q3) + lx2*sin(2*q2 + q3))
        
        C[1,0] = (q1d*(lx3**2*m3*sin(2*q2 + 2*q3) + lx2**2*m2*sin(2*q2) + lx2**2*m3*sin(2*q2) + 2*lx2*lx3*m3*sin(2*q2 + q3)))/2
        C[1,1] = 0
        C[1,2] = -lx2*lx3*m3*sin(q3)*(2*q2d + q3d)
        
        C[2,0] = lx3*m3*q1d*((lx3*sin(2*q2 + 2*q3))/2 + (lx2*sin(q3))/2 + (lx2*sin(2*q2 + q3))/2)
        C[2,1] = (lx2*lx3*m3*sin(q3)*(2*q2d + q3d))/2
        C[2,2] = -(lx2*lx3*m3*q2d*sin(q3))/2 
        
        return C
    
    
    ###########################################################################
    def B(self, q ):
        """ 
        Actuator Matrix  : dof x m
        """
        
        B = np.diag( np.ones( self.dof ) ) #  identity matrix
        
        return B
    
    
    ###########################################################################
    def g(self, q ):
        """ 
        Gravitationnal forces vector : dof x 1
        """
        
        [c1,s1,c2,s2,c3,s3,c12,s12,c23,s23] = self.trig( q )
        
        q2 = q[1]
        q3 = q[2]
        
        cos=np.cos
        
        lx2     = self.l2
        lx3     = self.l3
        
        m2      = self.m2
        m3      = self.m3
        
        G = np.zeros(3)
        
        
        g = self.gravity
        

        
        G[0] = 0
        G[1] = g*(m3*(lx3*cos(q2 + q3) + lx2*cos(q2)) + lx2*m2*cos(q2))
        G[2] = g*lx3*m3*cos(q2 + q3)

        return G
    
        
    ###########################################################################
    def d(self, q , dq ):
        """ 
        State-dependent dissipative forces : dof x 1
        """
        
        D = np.zeros((3,3))
        
        d = np.dot( D , dq )
        
        return d
    
        
    ###########################################################################
    # Graphical output
    ###########################################################################
    
    ###########################################################################
    def forward_kinematic_domain(self, q ):
        """ 
        """
        l = 1.2
        
        domain  = [ (-l,l) , (-l,l) , (0,l*2) ]#  
                
        return domain
    
    
    ###########################################################################
    def forward_kinematic_lines(self, q ):
        """ 
        Compute points p = [x;y;z] positions given config q 
        ----------------------------------------------------
        - points of interest for ploting
        
        Outpus:
        lines_pts = [] : a list of array (n_pts x 3) for each lines
        
        """
        
        lines_pts = [] # list of array (n_pts x 3) for each lines
        
        ###############################
        # ground line
        ###############################
        
        pts      = np.zeros(( 5 , 3 ))
        pts[0,:] = np.array([-1,-1,0])
        pts[1,:] = np.array([+1,-1,0])
        pts[2,:] = np.array([+1,+1,0])
        pts[3,:] = np.array([-1,+1,0])
        pts[4,:] = np.array([-1,-1,0])
        
        lines_pts.append( pts )
        
        ###########################
        # robot kinematic
        ###########################
        
        pts      = np.zeros(( 4 , 3 ))
        pts[0,:] = np.array([0,0,0])
        
        [c1,s1,c2,s2,c3,s3,c12,s12,c23,s23] = self.trig( q )
        
        # Three robot points

        lz1     = self.l1
        lx2     = self.l2
        lx3     = self.l3
        
        pts[1,0] = 0
        pts[1,1] = 0
        pts[1,2] = lz1
        
        pts[2,0] = lx2*c1*c2
        pts[2,1] = lx2*c1*s1
        pts[2,2] = lz1 + lx2*s2
        
        pts[3,0] = c1*(lx3*c23 + lx2*c2)
        pts[3,1] = s1*(lx3*c23 + lx2*c2)
        pts[3,2] = lz1 + lx3*s23 + lx2*s2
        
        lines_pts.append( pts )
                
        return lines_pts
    
    
class DrillingRobotOnJig( DrillingRobot ):
    
    
    ############################
    def __init__(self):
        """ """
        
        DrillingRobot.__init__( self )
        
        self.hole_position = np.array([0.25,0.25,0.4])
        self.hole_radius   = 0.05
        
    ##############################
    def f_ext(self, q , dq , t = 0 ):
        """ 
        External force due to contact during drilling
        
        """
        
        r  = self.forward_kinematic_effector( q )
        dr = self.forward_differential_kinematic_effector(q, dq)
        
        hole_position = self.hole_position
        hole_radius   = self.hole_radius
        
        # Contact:
        if r[2] < self.hole_position[2] :
            
            # Dans le bois
            
            fx = - dr[0] * 1000 # damping lateral
            fy = - dr[1] * 1000 # damping lateral
            fz = - dr[2] * 100 # damping vertical 
            
            # Pointe de la mèche dans le pré-trou
            if  (( r[0] > hole_position[0] - hole_radius ) &
                 ( r[0] < hole_position[0] + hole_radius ) &
                 ( r[1] > hole_position[1] - hole_radius ) &
                 ( r[1] < hole_position[1] + hole_radius ) ) :
                
                fz = fz /10 
            
            if r[2] < (self.hole_position[2] - 0.2) :
                
                # Dans l'acier
                
                fx = - dr[0] * 10000 # damping lateral
                fy = - dr[1] * 10000 # damping lateral
                fz = - dr[2] * 10000 # damping vertical 
            
            f_ext = np.array([fx,fy,fz])
            
        else:
            
            # No contact
            
            f_ext = np.zeros( self.e )
        
        return f_ext
    
    
    ###########################################################################
    def forward_kinematic_lines(self, q ):
        
        ###########################
        # Base graphic
        ###########################
        
        lines_pts = DrillingRobot.forward_kinematic_lines(self, q)
        
        ###########################
        # Drill
        ###########################
        
        [c1,s1,c2,s2,c3,s3,c12,s12,c23,s23] = self.trig( q )
        
        pts      = np.zeros(( 2 , 3 ))
        
        lz1     = self.l1
        lx2     = self.l2
        lx3     = self.l3
        
        pts[0,0] = c1*(lx3*c23 + lx2*c2)
        pts[0,1] = s1*(lx3*c23 + lx2*c2) 
        pts[0,2] = lz1 + lx3*s23 + lx2*s2 
        
        pts[1,0] = c1*(lx3*c23 + lx2*c2)
        pts[1,1] = s1*(lx3*c23 + lx2*c2) 
        pts[1,2] = lz1 + lx3*s23 + lx2*s2 - 0.4
        
        lines_pts.append(pts)
        
        ###########################
        # Hole
        ###########################
        
        pts      = np.zeros(( 2 , 3 ))
        
        x = self.hole_position[0]
        y = self.hole_position[1]
        z = self.hole_position[2]
        
        pts[0,:] = np.array([x,y,z-0.4])
        pts[1,:] = np.array([x,y,z-0.6])
        
        lines_pts.append( pts )
        
        
        return lines_pts
    
    
    
'''
#################################################################
##################          Main                         ########
#################################################################
'''


if __name__ == "__main__":
    
    
    sys = DrillingRobot()
    sys.x0 = np.array([0,0,0.8,0,0,0])
    sys.compute_trajectory()
    sys.plot_trajectory()
    sys.animate_simulation( is_3d = True )
        