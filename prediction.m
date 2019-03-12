function [tpred,jpred,xpred] = prediction(x0,ustar,tj,gamma,lambda,MaxStep,RelTol,rule)

eps = 5*MaxStep;

N = length(tj)-1;

% Simulate until right after the first jump
T = tj(2)-tj(1)+eps;
J = 1;
if isempty(ustar(1))
    u = 0;
else
    u = ustar(1);
end


j = []; % needed to prevent clashes with the built-in j function and j from Simulink to workspace

% use function workspace (Simulink normally uses base workspace)
% since x0 and u are redefined in the code
options = simset('SrcWorkspace','current');
sim('Example1_3.slx',[],options)
    
k = find(j,1);
if isempty(k)
    k = length(j);
end
tpred = t(1:k);
jpred = j(1:k);
xpred = x(1:k,:);
x0 = xpred(end,:);

% Simulate until right after the next jump, concatenate solutions
for idx = 2:N
    T = tj(idx+1)-tj(idx)+eps;
%     J = 1;
    if (idx == N)
        u = 0;
    else
        u = ustar(idx);
    end
    options = simset('SrcWorkspace','current');
    sim('Example1_3.slx',[],options)
    
    k = find(j,1);
    tpred = [tpred; tpred(end)+t(2:k)];
    jpred = [jpred; jpred(end)+j(2:k)];
    xpred = [xpred; x(2:k,:)];
    x0 = xpred(end,:);
end

end