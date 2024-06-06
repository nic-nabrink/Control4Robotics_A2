n = 100;
nr_test = 20;

task_lqr.cost.params.Q_s = diag([1, 1]);
task_lqr.cost.params.R_s = 20;


Reached = zeros(nr_test, nr_test);
Cost_M = zeros(nr_test, nr_test);
y_id=0;

for y0 = linspace(-10,n,nr_test)
    h_id = 0;
    y_id = y_id +1;
    for h0 = linspace(-10,n,nr_test)
            h_id=h_id+1;
            task_lqr.start_x = [y0; h0];

            sim_out_lqr = mobile_robot_sim(model, task_lqr, controller_lqr);
            % fprintf('--- LQR ---\n\n');
            % fprintf('trajectory cost: %.2f\n', sim_out_lqr.cost);
            % fprintf('target state [%.3f; %.3f]\n', task_lqr.goal_x);
            % fprintf('reached state [%.3f; %.3f]\n', sim_out_lqr.x(:,end));
          
            Cost_M(y_id, h_id) = sim_out_lqr.cost;
            Reached(y_id, h_id) = sim_out_lqr.x(1,end);

    end
end

figure(1);
tiledlayout('vertical')
surf(linspace(-10,n,nr_test), linspace(-10,n,nr_test), Cost_M)
xlabel('y0')
ylabel('h0')
figure(2);
surf(linspace(-10,n,nr_test), linspace(-10,n,nr_test), Reached)
xlabel('y0')
ylabel('h0')