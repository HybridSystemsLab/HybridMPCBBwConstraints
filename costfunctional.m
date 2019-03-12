function J = costfunctional(x0,u,gamma,lambda,delta_pred,theta,h)

% % Error handling
% if ~(isvector(x0) && isvector(u))
%     error('Initial condition x0 and input u must be vectors.')
% elseif length(x0)~=2
%     error('Initial condition x0 must be 2 by 1.')
% elseif length(u)<=1
%     error('Input vector u must have at least 2 elements.')
% elseif (x0(1)<0)
%     error('x0(1) must be nonnegative.')    
% elseif gamma <= 0
%     error('Gravity gamma must be positive.')
% elseif lambda<0
%     error('Coefficient of restitution lambda must be nonnegative.')
% elseif delta_pred<=0
%     error('Horizon parameter delta_pred must be positive.')
% elseif theta <= 0
%     error('Error: cost parameter theta must be positive.')
% elseif h < 0
%     error('Error: height parameter h must be nonnegative.')
% end

[tj,y,z] = discretetrajectory(x0,u,gamma,lambda,delta_pred);

n = length(tj)-1;

J = 0;

for idx = 1:n-1
    J = J+flowcost(y(1,idx),y(2,idx),gamma,theta,h)*(tj(idx+1)-tj(idx))+jumpcost(z(2,idx),gamma,lambda,theta,h);
end

J = J+flowcost(y(1,n),y(2,n),gamma,theta,h)*(tj(n+1)-tj(n))+terminalcost(z(1,n),z(2,n),gamma,theta,h);

end


