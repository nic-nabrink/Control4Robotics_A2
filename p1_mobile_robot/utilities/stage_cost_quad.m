function [g_k,q_k,Q_k,r_k,R_k] = stage_cost_quad(Q_s, R_s, x_goal, dt, x_k, u_k)
    g_k = 1/2 * (x_k - x_goal)' * Q_s * (x_k - x_goal) * dt;
    q_k = Q_s * (x_k - x_goal);
    Q_k = Q_s;
    r_k = R_s * u_k;
    R_k = R_s;
end