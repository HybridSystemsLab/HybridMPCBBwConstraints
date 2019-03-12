clear all
close all
clc

addpath('./Example_1.3-Bouncing_Ball_with_Input/');

% Parameters specified below for easy access, but initialize the Simulink model
% from file just in case 
initialization_ex1_3;   % Example_1.3-Bouncing_Ball_with_Input\initialization_ex1_3.m to

%% Physical variables

gamma = 9.81;   % gravity constant          - (0<gamma )
lambda = 0.8;   % restitution coefficent    - (0<=lambda<1)

%% MPC parameters

% prediction horizon parameters                                                   
tau_pred = 5;  % must be an integer
delta_pred = 0.4; % (0<delta_pred)

% cost function parameter
theta =(2/pi)*(1-lambda^4)/(1+lambda^4)*rand(1);

% desired height
h = 2.0;    % 0<=gamma

%% Simulation parameters 

% simulation horizon                                                    
T = 10;                                                                 
J = 2;                                                                 
                                                                        
% rule for jumps                                                        
rule = 1;   % prioritize jumps                                                                                                       
                                                                        
% solver tolerances
RelTol = 1e-8;
MaxStep = .005;

% tolerance parameter for prediction.m
eps = MaxStep;

%%

N = 20;
Ts = zeros(1,N);
Js = zeros(1,N);
TJs = zeros(1,N);
% initial condition
x0 = [0; 0];  % (0<=x0(1))
% x0 = [1; -1];  % (0<=x0(1))
% x0 = [3; 4];  % (0<=x0(1))

% initial guesses for the optimal control
u0 = zeros(1,tau_pred);

% solve optimal control problem
[ustar,Jstar] = solveOCP(x0,u0,gamma,lambda,delta_pred,theta,h);
clc

% Apply the optimal control on the system -- the function prediction
% uses tj to speed up simulations
[tj,y,z] = discretetrajectory(x0,ustar,gamma,lambda,delta_pred);
[tpred,jpred,xpred] = prediction(x0,ustar,tj,gamma,lambda,MaxStep,RelTol,rule);

k = find(jpred,1);
if isempty(k)
    k = length(jpred);
end
t = tpred(1:k);
j = jpred(1:k);
x = xpred(1:k,:);
x0 = x(end,:);

Ts(1) = t(end);
Js(1) = j(end);
TJs(1) = length(t);

for idx = 1:(N-1)

    % solve optimal control problem
    [ustar,Jstar] = solveOCP(x0,u0,gamma,lambda,delta_pred,theta,h);
    clc

    % Apply the optimal control on the system -- the function prediction
    % uses tj to speed up simulations
    [tj,y,z] = discretetrajectory(x0,ustar,gamma,lambda,delta_pred);
    [tpred,jpred,xpred] = prediction(x0,ustar,tj,gamma,lambda,MaxStep,RelTol,rule);
    
    k = find(jpred,1);
    if isempty(k)
        k = length(jpred);
    end
    t = [t; t(end)+tpred(2:k)];
    j = [j; j(end)+jpred(2:k)];
    x = [x; xpred(2:k,:)];
    x0 = x(end,:);
    
    Ts(idx+1) = t(end);
    Js(idx+1) = j(end);
    TJs(idx+1) = length(t);
end

W = gamma*x(:,1)+x(:,2).^2/2;

%%

figure(1), hold on, grid on
plot(t,x(:,1),'b')
plot(Ts,x(TJs,1),'ks','MarkerSize',4)
xlabel('$t$','Interpreter','latex'), ylabel('$x_1$','Interpreter','latex')
set(gca,'FontName','Times','FontSize',12)

figure(2), hold on, grid on
plot(t,W,'b'), hold on,
xlabel('$t$','Interpreter','latex'), ylabel('$W(x)$','Interpreter','latex')
set(gca,'FontName','Times','FontSize',12)