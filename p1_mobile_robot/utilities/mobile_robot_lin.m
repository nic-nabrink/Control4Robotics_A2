function [A_k,B_k] = mobile_robot_lin(xk,uk,dt,v)
    A_k = [1 dt*v*cos(xk(2)); 0 1];
    B_k = [0; dt];
end