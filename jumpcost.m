function LD = jumpcost(x2,gamma,lambda,theta,h)

    LD = (1-theta*pi/2)*gamma*h*(x2+sqrt(2*gamma*h))^2/2;
    
    if x2 <= -(sqrt(2*gamma*h)/lambda)
        LD = min(LD,(1-theta*pi/2)*(x2^2/2-gamma*h)^2-(1+theta*pi/2)*(lambda^2*x2^2/2-gamma*h)^2);
    end
    
end