function V = terminalcost(x1,x2,gamma,theta,h)

     V = (1+theta*atan(x2))*(gamma*(x1-h)+x2^2/2)^2;

end