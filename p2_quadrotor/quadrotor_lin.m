function [A_k, B_k] = quadrotor_lin(A_lin, B_lin, dt)
    A_k = A_lin*dt + eye([size(A_lin*dt)]);
    B_k = B_lin*dt;
end