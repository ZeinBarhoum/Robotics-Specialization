waypoints = [0    0   0;
             1    1   1;
             2    0   2;
             3    -1  1;
             4    0   0]';
[A,B,T,S] = test_traj(waypoints);


function [A,B,T,S] = test_traj(waypoints)
    d = waypoints(:,2:end) - waypoints(:,1:end-1);
    T = 2 * sqrt(d(1,:).^2 + d(2,:).^2 + d(3,:).^2);
    S = [0, cumsum(T)];
    waypoints0 = waypoints;

    syms c [3,4,4]
    syms t 
    syms p [3,4]
    conds = [];
    condsB = [];
    for i=1:4
        p(:,i) = c(:,i,1) + c(:,i,2)*(t - S(i))/(T(i)) + c(:,i,3)*((t - S(i))/(T(i)))^2 + c(:,i,4)*((t - S(i))/(T(i)))^3;
        p(:,i) = c(:,i,1) + c(:,i,2)*(t - S(i)) + c(:,i,3)*(t - S(i))^2 + c(:,i,4)*(t - S(i))^3;
        conds = [conds; subs(p(:,i),t,S(i))];
        condsB = [condsB;waypoints0(:,i)];

        conds = [conds; subs(p(:,i),t,S(i+1))];
        condsB = [condsB;waypoints0(:,i+1)];
        
    end

    for i=1:3
        conds = [conds; subs(diff(p(:,i),t),t,S(i+1)) - subs(diff(p(:,i+1),t),t,S(i+1))];
        condsB = [condsB;zeros(3,1)];

        conds = [conds; subs(diff(p(:,i),t,2),t,S(i+1)) - subs(diff(p(:,i+1),t,2),t,S(i+1))];
        condsB = [condsB;zeros(3,1)];
    end
    
    conds = [conds; subs(diff(p(:,1),t),t,S(1))];
    condsB = [condsB;zeros(3,1)];
%     conds = [conds; subs(diff(p(:,1),t,2),t,S(1)) == zeros(3,1)];

    conds = [conds; subs(diff(p(:,4),t),t,S(5))];
    condsB = [condsB;zeros(3,1)];
%     conds = [conds; subs(diff(p(:,4),t,2),t,S(5)) == zeros(3,1)]
    
    cof = reshape(c,[48,1]);
    
    A = eye(48);

    for i=1:48
        for j=1:48
            A(i,j) = diff(conds(i), cof(j));
        end
    end

    B = condsB;
end