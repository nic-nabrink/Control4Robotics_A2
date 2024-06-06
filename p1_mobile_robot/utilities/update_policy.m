function [theta_kff,theta_kfb,s_k_,s_k,S_k] = update_policy(A_k,B_k,g_k,q_k,Q_k,r_k,R_k,P_k,s_k1_,s_k1,S_k1,x_k,u_k)
    l_k = r_k + B_k'*s_k1;
    G_k = B_k'*S_k1*A_k;
    H_k = R_k+B_k'*S_k1*B_k;
    h = inv(H_k);
    theta_kff = u_k-h*l_k+h*G_k*x_k;
    theta_kfb = -h*G_k;

    K_k = -h*G_k;
    du_kff = -h*l_k;

    s_k_ = g_k + s_k1_ + 1/2 * (du_kff'*H_k*du_kff) + du_kff'*l_k;
    s_k = q_k + A_k'*s_k1 + K_k'*H_k*du_kff + K_k'*l_k + G_k'*du_kff;
    S_k = Q_k + A_k'*S_k1*A_k + K_k'*H_k*K_k + K_k'*G_k + G_k'*K_k;
end