function [g_N,q_N,Q_N] = terminal_cost_quad(Q_t,x_goal,x_N)
    g_N = 1/2 * (x_N - x_goal)' * Q_t * (x_N - x_goal);
    q_N = Q_t * (x_N - x_goal);
    Q_N = Q_t;
end