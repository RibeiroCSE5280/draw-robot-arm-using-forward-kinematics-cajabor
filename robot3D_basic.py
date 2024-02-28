#!/usr/bin/env python
# coding: utf-8


from vedo import *

def RotationMatrix(theta, axis_name):
  """ calculate single rotation of $theta$ matrix around x,y or z
              code from: https://programming-surgeon.com/en/euler-angle-python-en/
    input
        theta = rotation angle(degrees)
        axis_name = 'x', 'y' or 'z'
    output
        3x3 rotation matrix
    """
  c = np.cos(theta * np.pi / 180)
  s = np.sin(theta * np.pi / 180)
	
  if axis_name =='x':
    rotation_matrix = np.array([[1, 0,  0],
                                  [0, c, -s],
                                    [0, s,  c]])
  if axis_name =='y':
        rotation_matrix = np.array([[ c,  0, s],
                                    [ 0,  1, 0],
                                    [-s,  0, c]])
  elif axis_name =='z':
        rotation_matrix = np.array([[c, -s, 0],
                                    [s,  c, 0],
                                    [0,  0, 1]])
  return rotation_matrix

def getCylinderFrame(length):
    sphere_radius = 0.4
    sphere = Sphere(r=sphere_radius).pos(0,0,0).color("gray").alpha(.8)
    frameArrows = createCoordinateFrameMesh()

    mesh = Cylinder(r=sphere_radius, 
	                      height=length, 
	                      pos = ((length/2)+sphere_radius,0,0),
	                      c="aqua", 
	                      alpha=.8, 
	                      axis=(1,0,0)
	                      )
	
	# Combine all parts into a single object 
    frame = sphere + frameArrows + mesh 
    return frame


def forward_kinematics(angles, l1, l2, l3, l4):
    sphere_radius = 0.4

    # Matrix of Frame 1 (written w.r.t. Frame 0, which is the previous frame) 
    R_01 = RotationMatrix(angles[0], axis_name = 'z')   
    t_01   = np.array([[3],[3], [l1/2]])              
    T_01 = getLocalFrameMatrix(R_01, t_01)         
	
	


    # Matrix of Frame 2 (written w.r.t. Frame 1, which is the previous frame) 	
    R_12 = RotationMatrix(angles[1], axis_name = 'y')   # Rotation matrix
    t_12   = np.array([[0],[0], [(l1/2)+sphere_radius]])           # Frame's origin (w.r.t. previous frame)
	
	# Matrix of Frame 2 w.r.t. Frame 1 
    T_12 = getLocalFrameMatrix(R_12, t_12)
	# Matrix of Frame 2 w.r.t. Frame 0 (i.e., the world frame)
    T_02 = T_01 @ T_12
	
    frame2 = getCylinderFrame(l2)
    frame2.apply_transform(T_02) 

    R_23 = RotationMatrix(angles[2], axis_name = 'y')   # Rotation matrix
    t_23   = np.array([[l2+(2*sphere_radius)],[0.0], [0.0]])           # Frame's origin (w.r.t. previous frame)
	
	# Matrix of Frame 3 w.r.t. Frame 2 
    T_23 = getLocalFrameMatrix(R_23, t_23)
	
	# Matrix of Frame 3 w.r.t. Frame 0 (i.e., the world frame)
    T_03 = T_02 @ T_23 

    frame3 = getCylinderFrame(l3)
    frame3.apply_transform(T_03)

    R_34 = RotationMatrix(angles[3], axis_name = 'y')   # Rotation matrix
    t_34  = np.array([[l3+(2*sphere_radius)],[0.0], [0.0]])           # Frame's origin (w.r.t. previous frame)

    
    T_34 = getLocalFrameMatrix(R_34, t_34)
    T_04 = T_03 @ T_34

    
    frame4 = getCylinderFrame(l4)
    frame4.apply_transform(T_04)

    R_45 = RotationMatrix(0, axis_name = 'y')
    t_45 = np.array([[l4+(sphere_radius)],[0.0], [0.0]]) 
    T_45 = getLocalFrameMatrix(R_45, t_45)
    end_effector = T_04 @ T_45
    
    return T_01, T_02, T_03, T_04, end_effector

def createCoordinateFrameMesh():
    """Returns the mesh representing a coordinate frame
    Args:
      No input args
    Returns:
      F: vedo.mesh object (arrows for axis)
      
    """         
    _shaft_radius = 0.05
    _head_radius = 0.10
    _alpha = 1
    
    
    # x-axis as an arrow  
    x_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(1, 0, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='red',
                        alpha=_alpha)

    # y-axis as an arrow  
    y_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 1, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='green',
                        alpha=_alpha)

    # z-axis as an arrow  
    z_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 0, 1),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='blue',
                        alpha=_alpha)
    
    originDot = Sphere(pos=[0,0,0], 
                       c="black", 
                       r=0.10)


    # Combine the axes together to form a frame as a single mesh object 
    F = x_axisArrow + y_axisArrow + z_axisArrow + originDot
        
    return F


def getLocalFrameMatrix(R_ij, t_ij): 
    """Returns the matrix representing the local frame
    Args:
      R_ij: rotation of Frame j w.r.t. Frame i 
      t_ij: translation of Frame j w.r.t. Frame i 
    Returns:
      T_ij: Matrix of Frame j w.r.t. Frame i. 
      
    """             
    # Rigid-body transformation [ R t ]
    T_ij = np.block([[R_ij,                t_ij],
                     [np.zeros((1, 3)),       1]])
    
    return T_ij
	

def main():
    
    angles = [[0, -30, 20, -60],
              [30, -55, 40, -80],
              [50, -70, 60, -90]]
    l1, l2, l3, l4 = 1.8, 6, 4, 4

    # Set the limits of the graph x, y, and z ranges
    axes = Axes(xrange=(0,20), yrange=(-2,10), zrange=(0,6))

    
    for phi in angles:
        T_01, T_02, T_03, T_04, e = forward_kinematics(phi, l1, l2, l3, l4)

        base_frameArrows = createCoordinateFrameMesh()
        base_mesh = Cube(pos=(0, 0, 0), side= l1)
        base_frame = base_frameArrows + base_mesh
        base_frame.apply_transform(T_01)

        frame2=  getCylinderFrame(l2)
        frame2.apply_transform(T_02)

        frame3=  getCylinderFrame(l3)
        frame3.apply_transform(T_03)

        frame4 =  getCylinderFrame(l4)
        frame4 .apply_transform(T_04)

        end_effector = createCoordinateFrameMesh()
        end_effector.apply_transform(e)


        # Show everything 
        show([base_frame, frame2, frame3, frame4, end_effector], axes, viewup="z").close()


if __name__ == '__main__':
    main()



