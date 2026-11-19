%% =======================================================================
%  ARDrone Simulation Example: Trajectory Tracking 
%  =======================================================================
%  
%  The simulation is used to validate the trajectory tracking controller
%  and generation of the desired trajectory for the ARDRone before flight 
%  testing. Control and guidance blocks are
%  exactly the same for both simulation and real-time Wi-Fi control.
%  
%  =======================================================================

%%
%  Cleaning workspace
bdclose all;
clear all;
clc

%%
% Adding ARDrone library path 
addpath ../lib; 
%% Simulation parameters

% Flight management system sample time. This is the sample time at which
% the control law is executed. 
FMS.Ts = 0.03;%0.065; 

% Time delay due to communication between drone and host computer
timeDelay = FMS.Ts*4; 


%% Vehicle model based on linear dynamics

% Loading state space representation of vehicle dynamics
setupARModel; 

%%
% Loading list of waypoints
% waypoints = getWaypoints() ;


%% 
% Simulation time
simDT = 0.005 ;

%%
% Loading Simulink model of ARDrone
ARDroneTTSim ;



