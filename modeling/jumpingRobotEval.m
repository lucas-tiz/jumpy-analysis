% test JumpingRobot object
clear, clc

repo_folder = 'jumpy-analysis';
current_path = pwd;
idx_repo = strfind(pwd, repo_folder);
repo_path = current_path(1:(idx_repo-1 + length(repo_folder)));
addpath(fullfile(repo_path, 'modeling'))


config_file = fullfile(repo_path, 'modeling', 'robot_config_default.yaml');

tic
robot = JumpingRobot(config_file);
toc

tic
robot2 = copy(robot);
toc


% dispRobotParams(robot)
% dispRobotJoint(robot.joints.knee_right)
% dispRobotJoint(robot.joints.hip_left)
% robot.spatialRobot.I{5}


% config.cam.d = 123;
% config.cam.phi_range = [10 30];
% config.cam.beta_range = [20 40];
% config.knee.slope = 987;
% config.hip.theta_range = [50 80];
% config.cam.beta_inc = 5;

config.model.joint_damp = [1,2,3,4];
config.state0.q0 = [5,6,7,8,9];
config.control.t_musc_activate = [2,4,5,6];

config.morphology.l = [1,2,3,4,5];
config.morphology.m = [1,1,1,1,1];

tic
robot.updateConfig(config);
toc

% dispRobotParams(robot)
% dispRobotJoint(robot.joints.knee_right)
% dispRobotJoint(robot.joints.hip_left)
% robot.spatialRobot.I{5}




% robot2.config.hip.rad0 = 456;
% robot.config.hip.rad0
% 
% robot2.joints.knee_left.cam_param.d = 123;
% robot.joints.knee_left.cam_param.d




modifyTestProp(robot, 3)




%% Functions

function dispRobotParams(robot)
    disp(robot.config.cam)
    disp(robot.config.knee)
    disp(robot.config.hip)
    disp(robot.config.model)
    disp(robot.config.morphology)
    disp(robot.config.state0)
    disp(robot.config.control)

end

function dispRobotJoint(joint)
    disp(joint.joint_param)
    disp(joint.cam_param)
end

function modifyTestProp(robot, n)
    robot.test_prop = n;
end
