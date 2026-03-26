% Template to visualize in V-REP the inverse kinematics algorithm developed
%          for the kinova Jaco 2 7-DOF robot
%
% Read Instructions.odt first !
% 
% Do not modify any part of this file except the strings within
%    the symbols << >>
%
% G. Antonelli, Introduction to Robotics, 2022

%
% simRemoteApi.start(19999)
%



function q_jaco = main_template(q)
    close all
    clc
    addpath('functions_matlab');
    run('InverseKinematics2.m');
    addpath('functions_coppelia/');
    addpath('functions_matlab/');
    porta = 19999;          % default V-REP port

    

    % decimazione per visualizzazione rapida
    q = downsample(q',4)';
    
    [n,N]  = size(q);         % number of points of the simulation 
    q_jaco = zeros(n,N);    % q_jaco(:,i) collects the joint position for t(i) sent to Jaco
    %q(:,1) = [77  -17 0  43 -94 77 71]'/180*pi; % approximated home configuration
    q_jaco(:,1) = mask_q_DH2Jaco(q(:,1));
    
   
    clc
    fprintf('----------------------');
    fprintf('\n simulation started ');
    fprintf('\n trying to connect...\n');
    [clientID, vrep ] = StartVrep(porta);
  
   
    handle_joint = my_get_handle_Joint(vrep,clientID);      % handle to the joints
    my_set_joint_target_position(vrep, clientID, handle_joint, q_jaco(:,1)); % first move to q0
    q_act = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep     
        
    % main simulation loop
    for i=1:N   
        % DH -> Kinova conversion
        q_jaco(:,i) = mask_q_DH2Jaco(q(:,i));
        my_set_joint_target_position(vrep, clientID, handle_joint, q_jaco(:,i));
        % required only for tempification
        q_act = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep     
        % Kinova -> DH conversion
        q_jaco(:,i) = mask_q_Jaco2DH(q_act);       
    end
    
    DeleteVrep(clientID, vrep); 
            
end

% constructor
function [clientID, vrep ] = StartVrep(porta)

    vrep = remApi('remoteApi');   % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1);        % just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1',porta,true,true,5000,5);% start the simulation
    
    if (clientID>-1)
        disp('remote API server connected successfully');
    else
        disp('failed connecting to remote API server');   
        DeleteVrep(clientID, vrep); %call the destructor!
    end
    % to change the simulation step time use this command below, a custom dt in v-rep must be selected, 
    % and run matlab before v-rep otherwise it will not be changed 
    % vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, 0.002, vrep.simx_opmode_oneshot_wait);
    vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
end  

% destructor
function DeleteVrep(clientID, vrep)
    
    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait); % pause simulation
    %vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait); % stop simulation
    vrep.simxFinish(clientID);  % close the line if still open
    vrep.delete();              % call the destructor!
    disp('simulation ended');
    
end

function my_set_joint_target_position(vrep, clientID, handle_joint, q)
           
    [m,n] = size(q);
    for i=1:n
        for j=1:m
            err = vrep.simxSetJointPosition(clientID,handle_joint(j),q(j,i),vrep.simx_opmode_oneshot);
            if (err ~= vrep.simx_error_noerror)
                fprintf('failed to send joint angle q %d \n',j);
            end
        end
    end
    
end

function handle_joint = my_get_handle_Joint(vrep,clientID)

    [~,handle_joint(1)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_1',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(2)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_2',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(3)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_3',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(4)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_4',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(5)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_5',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(6)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_6',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(7)] = vrep.simxGetObjectHandle(clientID,'Revolute_joint_7',vrep.simx_opmode_oneshot_wait);

end

function my_set_joint_signal_position(vrep, clientID, q)
           
    [~,n] = size(q);
    
    for i=1:n
        joints_positions = vrep.simxPackFloats(q(:,i)');
        [err]=vrep.simxSetStringSignal(clientID,'jointsAngles',joints_positions,vrep.simx_opmode_oneshot_wait);

        if (err~=vrep.simx_return_ok)   
           fprintf('failed to send the string signal of iteration %d \n',i); 
        end
    end
    pause(8);% wait till the script receives all data, increase it if dt is too small or tf is too high
    
end


function angle = my_get_joint_target_position(clientID,vrep,handle_joint,n)
    
    for j=1:n
         vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_streaming);
    end

    pause(0.05);

    for j=1:n          
         [err(j),angle(j)]=vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_buffer);
    end

    if (err(j)~=vrep.simx_return_ok)   
           fprintf(' failed to get position of joint %d \n',j); 
    end

end

