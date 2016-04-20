#!/usr/bin/env python

import openravepy
import adapy
import prpy
import argparse
import rospy
import numpy as np
import time
import timeit

import IPython




def ang_error_quats(q1, q2):
    quat_between = openravepy.quatMultiply( openravepy.quatInverse(q1), q2)
    w = min(quat_between[0], 1.-1e-4)
    w = max(w, -(1.-1e-4))
    return 2.*np.arccos(w)

def ang_error_mats(m1, m2):
    return ang_error_quats(openravepy.quatFromRotationMatrix(m1), openravepy.quatFromRotationMatrix(m2))

def dist_error(p1, p2):
    return np.linalg.norm(p1-p2)


def calculate_errors(robot, manip, poses, dof_values_init, dof_values_found):
    curr_dof_vals = robot.GetDOFValues()
    robot.SetActiveDOFs(manip.GetIndices())

    pos_errors = []
    ang_errors = []
    dof_errors = []
    for dof_vals,pose,dof_init in zip(dof_values_found, poses, dof_values_init):
        if dof_vals is not None:
            robot.SetActiveDOFValues(dof_vals)
            pose_sol = manip.GetEndEffectorTransform()

            pos_errors.append(dist_error(pose_sol[0:3,3], pose[0:3,3]))
            ang_errors.append(ang_error_mats(pose_sol, pose))
            dof_errors.append(dist_error(dof_init, dof_vals))


    robot.SetDOFValues(curr_dof_vals)

    return pos_errors, ang_errors, dof_errors
    

def generate_random_dofvals_and_transforms(robot, manip, num_random=10000):
    #set sampling limits
    lower, upper = robot.GetDOFLimits()

    manip_indices = manip.GetIndices()
    robot.SetActiveDOFs(manip_indices)

    #for continuous joints, set value between -2*pi and 2*pi
    for idx in manip_indices:
        joint = robot.GetJointFromDOFIndex(idx)
        if joint.IsCircular(0):
            lower[idx] = -2.*np.pi
            upper[idx] = 2.*np.pi

    #keep only manip indices
    lower = lower[manip_indices]
    upper = upper[manip_indices]

    #generate dofs, set robot, save pose
    dofs = []
    poses = []
    while len(dofs) < num_random:
        dof = np.random.uniform(lower, upper)
        robot.SetActiveDOFValues(dof)
        
        if robot.CheckSelfCollision():
            continue;

        pose = robot.arm.GetEndEffectorTransform()

        dofs.append(dof)
        poses.append(pose)

    return dofs,poses


def test_curr_ik_solver(env, robot, manip, dof_vals, poses):
    with env:
        curr_dof_vals = robot.GetDOFValues()

        robot.SetActiveDOFs(manip.GetIndices())

        num_correct = 0
        dof_sols = []
        dof_init = []
 
        start_time = time.clock()
        for dof_val,pose in zip(dof_vals,poses):
            dof_init.append(robot.GetActiveDOFValues())
            sol = robot.arm.FindIKSolution(pose, 0)
            dof_sols.append(sol)
            if sol is not None:
                num_correct += 1
            robot.SetActiveDOFValues(dof_val)

        end_time = time.clock()

        robot.SetDOFValues(curr_dof_vals)


    percent_correct = float(num_correct) / float(len(poses))
    tot_time = (end_time - start_time)/ float(len(poses))

    #get accuracy
    pos_errors, ang_errors, dof_diffs = calculate_errors(robot, manip, poses, dof_init, dof_sols)

    return percent_correct, tot_time, pos_errors, ang_errors, dof_diffs


        
if __name__ == "__main__":

    parser = argparse.ArgumentParser('Trac IK tester')
    parser.add_argument('-s', '--sim', action='store_true', default=True,
                        help='simulation mode')
    parser.add_argument('-v', '--viewer', nargs='?', const=True, default='',
                        help='attach a viewer of the specified type')
    args = parser.parse_args()
    adapy_args = {'sim':args.sim,
                'attach_viewer':args.viewer,
                }

    env, robot = adapy.initialize(**adapy_args)
    manip = robot.arm

    #put robot in home
    inds, pos = robot.configurations.get_configuration('home')
    with env:
        robot.SetDOFValues(pos, inds)

    cur_pos = manip.GetEndEffectorTransform()
    
    #T0 = robot.arm.GetEndEffectorTransform()
    #T0[0,3] += 0.1
    #print robot.arm.FindIKSolution(T0, 0)

    dof_vals, poses = generate_random_dofvals_and_transforms(robot, manip)


    #TEST TRAC IK
    ik = openravepy.RaveCreateIkSolver(env, 'TraciK')
    robot.arm.SetIKSolver(ik)
    trac_percent_correct, trac_tot_time, trac_pos_errors, trac_ang_errors, trac_dof_diff = test_curr_ik_solver(env, robot, manip, dof_vals, poses)
    print "Trac-IK results: "
    print "Portion solved: " + str(trac_percent_correct)
    print "Average time: " + str(trac_tot_time) + ' s'
    print "Avg dist err: " + str(np.mean(trac_pos_errors)) + ' +- ' + str(np.std(trac_pos_errors)) + ' m'
    print "Avg ang err: " + str(np.mean(trac_ang_errors)) + ' +- ' + str(np.std(trac_ang_errors)) + ' rad'
    print "Avg dof difference: " + str(np.mean(trac_dof_diff)) + ' +- ' + str(np.std(trac_dof_diff))

    print ""

    #TEST NLOPT IK
    ik = openravepy.RaveCreateIkSolver(env, 'NloptIK')
    robot.arm.SetIKSolver(ik)
    nlopt_percent_correct, nlopt_tot_time, nlopt_pos_errors, nlopt_ang_errors, nlopt_dof_diff = test_curr_ik_solver(env, robot, manip, dof_vals, poses)
    print "NLOPT results: "
    print "Portion solved: " + str(nlopt_percent_correct)
    print "Average time: " + str(nlopt_tot_time) + ' s'
    print "Avg dist err: " + str(np.mean(nlopt_pos_errors)) + ' +- ' + str(np.std(nlopt_pos_errors)) + ' m'
    print "Avg ang err: " + str(np.mean(nlopt_ang_errors)) + ' +- ' + str(np.std(nlopt_ang_errors)) + ' rad'
    print "Avg dof difference: " + str(np.mean(nlopt_dof_diff)) + ' +- ' + str(np.std(nlopt_dof_diff))



    IPython.embed()
