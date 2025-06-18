% System parameters
Jm = 2; Bm = 0.5;
Jl = 10; Bl = 1;
k = 100;

% State-space matrices
A = [  0        1        0        0;
     -k/Jm   -Bm/Jm     k/Jm     0;
        0       0        0        1;
      k/Jl     0      -k/Jl   -Bl/Jl];

B = [0; 1/Jm; 0; 0];
C = [0 0 1 0]; % Output = theta_l
D = 0;

t = linspace(0, 10, 1000);
r = ones(size(t)); % Step input

% Auto-tune Kp and Kd
found = false;
for Kp = 9
    for Kd = 8.0
        K = [0 0 Kp Kd];
        sys_cl = ss(A - B*K, B, C, D);
        [y, ~] = lsim(sys_cl, r, t);
        
        steady = mean(y(end-50:end));
        max_val = max(y);
        transient_error = abs(max_val - steady) / steady * 100;
        
        if transient_error <= 5
            best_Kp = Kp;
            best_Kd = Kd;
            best_error = transient_error;
            found = true;
            break;
        end
    end
    if found
        break;
    end
end

% Use final found values
if found
    disp("Result: TRUE (≤ 5% transient error)")
else
    disp("Result: FALSE (No suitable Kp, Kd found in range)")
end

disp(['Kp = ', num2str(best_Kp), ', Kd = ', num2str(best_Kd)])
disp(['Transient Error = ', num2str(best_error), '%'])

% Simulate with best gains
K = [0 0 best_Kp best_Kd];
sys_final = ss(A - B*K, B, C, D);
[y_final, t_final] = lsim(sys_final, r, t);

% Plot
figure
subplot(2,1,1)
plot(t_final, y_final - mean(y_final(end-50:end)), 'y')
title('Controlled Error (y - steady)')
grid on

subplot(2,1,2)
plot(t_final, y_final, 'y')
title('Output Response (theta_l)')
xlabel('Time (s)')
grid on
