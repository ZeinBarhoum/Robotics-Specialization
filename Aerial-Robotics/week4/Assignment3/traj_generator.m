function [ desired_state ] = traj_generator(t, state, waypoints)
% TRAJ_GENERATOR: Generate the trajectory passing through all
% positions listed in the waypoints list
%
% NOTE: This function would be called with variable number of input arguments.
% During initialization, it will be called with arguments
% trajectory_generator([], [], waypoints) and later, while testing, it will be
% called with only t and state as arguments, so your code should be able to
% handle that. This can be done by checking the number of arguments to the
% function using the "nargin" variable, check the MATLAB documentation for more
% information.
%
% t,state: time and current state (same variable as "state" in controller)
% that you may use for computing desired_state
%
% waypoints: The 3xP matrix listing all the points you much visited in order
% along the generated trajectory
%
% desired_state: Contains all the information that is passed to the
% controller for generating inputs for the quadrotor
%
% It is suggested to use "persistent" variables to store the waypoints during
% the initialization call of trajectory_generator.


%% Example code:
% Note that this is an example of naive trajectory generator that simply moves
% the quadrotor along a stright line between each pair of consecutive waypoints
% using a constant velocity of 0.5 m/s. Note that this is only a sample, and you
% should write your own trajectory generator for the submission.

% persistent waypoints0 T S P
% if nargin > 2
%     d = waypoints(:,2:end) - waypoints(:,1:end-1);
%     T = 5 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
%     S = [0, cumsum(T)];
%     waypoints0 = waypoints;
% 
%     syms c [3,4,4]
%     syms t 
%     syms p [3,4]
%     conds = [];
%     condsB = [];
%     for i=1:size(waypoints,2)-1 
%         p(:,i) = c(:,i,1) + c(:,i,2)*(t - S(i))/(T(i)) + c(:,i,3)*((t - S(i))/(T(i)))^2 + c(:,i,4)*((t - S(i))/(T(i)))^3;
% 
%         conds = [conds; subs(p(:,i),t,S(i))];
%         condsB = [condsB;waypoints0(:,i)];
% 
%         conds = [conds; subs(p(:,i),t,S(i+1))];
%         condsB = [condsB;waypoints0(:,i+1)];
%         
% 
%     end
%     for i=1:3
%         conds = [conds; subs(diff(p(:,i),t),t,S(i+1)) - subs(diff(p(:,i+1),t),t,S(i+1))];
%         condsB = [condsB;zeros(3,1)];
% 
%         conds = [conds; subs(diff(p(:,i),t,2),t,S(i+1)) - subs(diff(p(:,i+1),t,2),t,S(i+1))];
%         condsB = [condsB;zeros(3,1)];
%     end
%     
%     conds = [conds; subs(diff(p(:,1),t),t,S(1))];
%     condsB = [condsB;zeros(3,1)];
% %     conds = [conds; subs(diff(p(:,1),t,2),t,S(1)) == zeros(3,1)];
% 
%     conds = [conds; subs(diff(p(:,4),t),t,S(5))];
%     condsB = [condsB;zeros(3,1)];
% %     conds = [conds; subs(diff(p(:,4),t,2),t,S(5)) == zeros(3,1)]
%     
%     cof = reshape(c,[48,1]);
%     
%     A = eye(48);
% 
%     for i=1:48
%         for j=1:48
%             A(i,j) = diff(conds(i), cof(j));
%         end
%     end
%     
%     c_solved = inv(A) * condsB;
% 
%     P = subs(p, cof, c_solved);
% %     double(subs(p,t,1))
% 
% else
%     if(t > S(end))
%         t = S(end);
%     end
%     t_index = find(S >= t,1);
% 
%     if(t_index > 1)
%         t = t - S(t_index-1);
%     end
% 
%     if(t == 0)
%         desired_state.pos = waypoints0(:,1);
%     else
%         scale = t/T(t_index-1);
%         t
%         desired_state.pos = (1 - scale) * waypoints0(:,t_index-1) + scale * waypoints0(:,t_index);
%     end
% 
%     desired_state.vel = zeros(3,1);
%     desired_state.acc = zeros(3,1);
%     desired_state.yaw = 0;
%     desired_state.yawdot = 0;
% end
% %


%% Fill in your code here



persistent waypoints0 T S P
if nargin > 2
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    T = 1 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    S = [0, cumsum(T)];
    waypoints0 = waypoints;

    syms c [3,4,4]
    syms time
    syms p [3,4]
    conds = [];
    condsB = [];
    for i=1:size(waypoints,2)-1 
        p(:,i) = c(:,i,1) + c(:,i,2)*(time- S(i))/(T(i)) + c(:,i,3)*((time- S(i))/(T(i)))^2 + c(:,i,4)*((time- S(i))/(T(i)))^3;

        conds = [conds; subs(p(:,i),time,S(i))];
        condsB = [condsB;waypoints0(:,i)];

        conds = [conds; subs(p(:,i),time,S(i+1))];
        condsB = [condsB;waypoints0(:,i+1)];
        

    end
    for i=1:3
        conds = [conds; subs(diff(p(:,i),time),time,S(i+1)) - subs(diff(p(:,i+1),time),time,S(i+1))];
        condsB = [condsB;zeros(3,1)];

        conds = [conds; subs(diff(p(:,i),time,2),time,S(i+1)) - subs(diff(p(:,i+1),time,2),time,S(i+1))];
        condsB = [condsB;zeros(3,1)];
    end
    
    conds = [conds; subs(diff(p(:,1),time),time,S(1))];
    condsB = [condsB;zeros(3,1)];

    conds = [conds; subs(diff(p(:,4),time),time,S(5))];
    condsB = [condsB;zeros(3,1)];
    
    cof = reshape(c,[48,1]);
    
    A = eye(48);

    for i=1:48
        for j=1:48
            A(i,j) = diff(conds(i), cof(j));
        end
    end
    
    c_solved = inv(A) * condsB;

    P = subs(p, cof, c_solved);
    V = diff(P, time);
    A = diff(V, time);
    matlabFunction(P,'File','pos.m');
    matlabFunction(V,'File','vel.m');
    matlabFunction(A,'File','acc.m');
%     double(subs(p,t,1))

else
    flag = 0;
    if(t > S(end))
        t = S(end);
        flag = 1;
    end
    t_index = find(S >= t,1);

    if(t_index == 1)
        desired_state.pos = waypoints0(:,1);
        desired_state.vel = zeros(3,1);
        desired_state.acc = zeros(3,1);
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    else
        p = pos(t);
        v = vel(t);
        a = acc(t);
        desired_state.pos = p(:,t_index - 1);
        desired_state.vel = v(:,t_index - 1);
        desired_state.acc = a(:,t_index - 1);
        if(flag == 1)
            desired_state.vel = zeros(3,1);
            desired_state.acc = zeros(3,1);
        end

        
        desired_state.yaw = 0;
        desired_state.yawdot = 0;
    end
    
end
%
end

