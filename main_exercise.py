#!/usr/bin/env python

"""
Manipulator simulation setup
@author: Bijo Sebastian 
"""

#Import libraries
import time
import math
import numpy as np

#Import files
import sim_interface
import robot_params

def inv_kin(goal_position):
    #Compute joint angles [in radians] to reach desired position [in meters] 
    x_desired = goal_position[0]
    y_desired = goal_position[1]

    ### Fill this part ###
    
    print("Desired joint angles",[theta_1, theta_2])
    return [theta_1, theta_2]

def get_path(joint_angles_1, joint_angles_2):
    #Get path [in radians] that goes from current to new goal while avoiding obstacle (wall)
    
    ### Fill this part ###
    
    print ("path", joint_angles)
    return joint_angles

def get_jacobian(joint_angles):
    #Get 2x2 jacobian matrix for given joint angles [in radians] 
    
    ### Fill this part ###
    
    print("Jacobian", jacobian)
    return jacobian
    
def main():
    if (sim_interface.sim_init()):

        #Obtain handles to sim elements
        sim_interface.get_handles()

        #Start simulation
        if (sim_interface.start_simulation()):
            
            #Exercise 1
            #Go from home configuation to goal 1
            
            #Obtain goal_1 position
            goal_position = sim_interface.get_goal_position(1)
            
            #Compute joint angles to get to desired position
            joint_angles = inv_kin(goal_position)
            
            #Set joint angles 
            sim_interface.set_joint_position(joint_angles)
            time.sleep(0.5)
            
            #Obtain end effector position
            end_effector_position = sim_interface.get_end_effector_position()            
            
            #Verify
            difference_norm = math.fabs(goal_position[0] - end_effector_position[0]) + math.fabs(goal_position[1] - end_effector_position[1]) 
            if difference_norm < 0.001:
                print("Exercise 1 result: Success")
            else:
                print("Exercise 1 result: Failed")
                return
            
        
            #Exercise 2
            #Go from goal 1 to goal 2 avoiding the wall
            # To get the configuration space matrix
            #config_matrix = sim_interface.get_config_matrix()
            
            #Get current joint angles
            joint_angles_1 = sim_interface.get_joint_position()
            
            #Obtain goal_2 position
            goal_position = sim_interface.get_goal_position(2)
            
            #Compute joint angles to get to desired position
            joint_angles_2 = inv_kin(goal_position)
            
            #Get path that goes from current to new goal while avoiding obstacles
            joint_angles = get_path(joint_angles_1, joint_angles_2)
            
            for i in range(len(joint_angles)):
                #Set joint angles 
                sim_interface.set_joint_position(joint_angles[i])
                time.sleep(0.5)
                if sim_interface.collission_check():
                    print("Exercise 2 result: Failed")
                    return
                
            print("Exercise 2 result: Success")
            
            #Exercise 3
            #Move the manipulator along global x axis for 0.5m to reach goal 3
            
            #Obtain goal_3 position
            goal_position = sim_interface.get_goal_position(3)
                        
            for i in range(1,6,1):
                desired_joint_angles = []
                current_joint_angles = sim_interface.get_joint_position()
                jacobian = get_jacobian(current_joint_angles)
                joint_angle_rates = np.matmul(np.linalg.inv(jacobian),np.array([[0.1], [0.0]]))
                print("Joint angle rates", joint_angle_rates)
                desired_joint_angles.append(current_joint_angles[0] + joint_angle_rates[0,0])
                desired_joint_angles.append(current_joint_angles[1] + joint_angle_rates[1,0])
                sim_interface.set_joint_position(desired_joint_angles)
                time.sleep(1.0)
                
            #Obtain end effector position
            end_effector_position = sim_interface.get_end_effector_position()            
            
            #Verify
            difference_norm = math.fabs(goal_position[0] - end_effector_position[0]) + math.fabs(goal_position[1] - end_effector_position[1]) 
            if difference_norm < 0.1:
                print("Exercise 3 result: Success")
            else:
                print("Exercise 3 result: Failed")
                return

        else:
            print ('Failed to start simulation')
    else:
        print ('Failed connecting to remote API server')
    
    #Stop simulation
    sim_interface.sim_shutdown()
    time.sleep(0.5)
    return

#run
if __name__ == '__main__':

    main()                    
    print ('Program ended')
            

 