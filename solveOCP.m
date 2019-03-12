function [ustar,Jstar] = solveOCP(x0,u0,gamma,lambda,delta_pred,theta,h)

objective = @(u)costfunctional(x0,u,gamma,lambda,delta_pred,theta,h);
[ustar,Jstar] = fmincon(objective,u0);

% inputs are unconstrained in discretetrajectory.m (see file)
% take abs value to correct behavior 
ustar = abs(ustar);
end