function [tj,y,z] = discretetrajectory(x0,u,gamma,lambda,delta_pred)

% Given initial condition x(0,0) = x0 and vector of inputs u, with u(j) 
% corresponding to the input u(t_j,j-1) at hybrid time (t_j,j-1), compute t_j's,
% x(t_j,j)'s, and x(t_{j+1},j)'s.
%   
% u is replaced with abs(u) in the script, as the computation assumes
% inputs to be positive.
%
% u should have J elements, where J>=1 is the number of jumps. J should
% be chosen such that J = tau_pred, where tau_pred is the prediction
% horizon parameter, with delta_pred the corresponding parameter.



% % Error handling - uncomment if needed
% if ~(isvector(x0) && isvector(u))
%     error('Initial condition x0 and input u must be vectors.')
% elseif length(x0)~=2
%     error('Initial condition x0 must be 2 by 1.')
% elseif length(u)<=1
%     error('Input vector u must have at least 2 elements.')
% elseif ~((x0(1)>=0) && isempty(find(u<0,1)))
%     error('x0(1) and elements of u must be nonnegative.')    
% elseif gamma <= 0
%     error('Gravity gamma must be positive.')
% elseif lambda<0
%     error('Coefficient of restitution lambda must be nonnegative.')
% elseif delta_pred<=0
%     error('Horizon parameter delta_pred must be positive.')
% end


u = abs(u);         % make sure inputs are positive

J = length(u);      % number of jumps; i.e. tau_pred

tj = zeros(1,J+2);  % t_j's
y = zeros(2,J+1);   % x(t_j,j)'s (state at left endpoint of intervals of flow)
z = zeros(2,J+1);   % x(t_{j+1},j)'s (state at right endpoint of intervals of flow)

% Initialize
y(:,1) = x0;
tj(1) = 0;

for idx = 1:J+1
    
    % Check if y(1,i) belongs to jump set. Otherwise, compute when the next
    % impact is going to happen, and the state at the next impact. 
    if (y(1,idx)==0) && (y(2,idx)<=0)
        tj(idx+1) = tj(idx)+0;
        z(:,idx) = y(:,idx);
    elseif y(1,idx)>=0
        tj(idx+1) = tj(idx)+(y(2,idx)+sqrt(y(2,idx)^2+2*gamma*y(1,idx)))/gamma;   % See (1) below.
        z(:,idx) = [0; -sqrt(y(2,idx)^2+2*gamma*y(1,idx))];                   % See (2) below.
    end    
    
    % Compute post-impact velocity y(2,i+1), with the assumption that
    % inputs are constrained to be positive. This guarantees y(2,i+1)>=0
    if idx<=J
        y(2,idx+1) = -lambda*z(2,idx)+u(idx);    
    end
    
end

% check whether any t_j is beyond the horizon
idx = find(tj>=delta_pred*J,1);

if(isempty(idx))
else
    tj = tj(1:idx);
    y = y(:,(1:idx-1));
    z = z(:,(1:idx-1));
    
    % Recompute terminal point if necessary
    if tj(end)>delta_pred*J
        tj(end) = delta_pred*J;
        z(2,end) = y(2,end)-gamma*(tj(end)-tj(end-1));          % Integrate directly
        z(1,end) = (y(2,end)^2-z(2,end)^2)/(2*gamma)+y(1,end);  % See (2) below.
    end
end

end

% [1] Goebel, R., Sanfelice, R.G., and Teel, A.R. (2012). Hybrid Dynamical
%     Systems: Modeling, Stability, and Robustness. Princeton University Press,
%     Princeton, NJ.

% (0) The terminal time (T,J) of the trajectory has to be such that max(T/delta_pred,J) =tau_pred  
% (1) See Equation (2.3) in [1].
% (2) Computed by equating the total energies (kinetic+potential) in the
%     beginning and end of the interval of flow, as there is no dissipation
%     during flows.
