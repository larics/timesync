%% perform filtering with median and Holt-Winters
active_settings = 1;  % 1 - imu, 2 - camera

clock_readings = numbers_imu(:,2) - numbers_imu(1,2);
sensor_readings = numbers_imu(:,3) - numbers_imu(1,3);
current_hyp_t0 = clock_readings - sensor_readings;

switch active_settings
  case 1 % imu
    median_window_size = 1500;

    sensor_framerate = 200;

    alfa = 0.0004;
    beta = 0.0003;

    clamp_time = 120*200;
    early_alfa_start = 0.05;
    early_beta_start = 0.001;

    use_med_filtering = true;
    initial_b = -3.4e-7;
    
  case 2 % camera
    median_window_size = 2500;

    sensor_framerate = 20;

    alfa = 0.003;
    beta = 0.002;

    clamp_time = 500;
    early_alfa_start = 0.1;
    early_beta_start = 0.0;

    use_med_filtering = true;
    initial_b = -3e-7;
end

%%%%

n = size(current_hyp_t0, 1);
s = zeros(n, 1);
b = s;

if use_med_filtering
    med_filtered = medfilt(median_window_size, current_hyp_t0);
else
    med_filtered = current_hyp_t0; % skip filtering
end

% the median filter needs ~500 samples to stabilize itself, we clamp to it
% until this time passes

b(1)= initial_b;
s(1) = med_filtered(1);

for i=2:n
    if i < clamp_time
        progress=1-exp(0.5*(1-1/(1-i/clamp_time)));
        early_alfa= progress*alfa+(1-progress)*early_alfa_start;
        early_beta= progress*beta+(1-progress)*early_beta_start;
      s(i)=early_alfa*med_filtered(i)+(1-early_alfa)*(s(i-1)+b(i-1));
      b(i)=early_beta*(s(i)-s(i-1))+(1-early_beta)*b(i-1);
    else
      s(i)=alfa*med_filtered(i)+(1-alfa)*(s(i-1)+b(i-1));
      b(i)=beta*(s(i)-s(i-1))+(1-beta)*b(i-1);
    end
end

x = (1:n)/sensor_framerate;
figure(1);
hold off;
plot(x, current_hyp_t0);
hold on; 
if use_med_filtering
    plot(x, med_filtered);
end
plot(x, s);
%legend('original signal', 'median filtered (input)', 'Holt-Winters filtered')
figure(2); 
hold off;
plot(x(clamp_time:end), b(clamp_time:end));
title('estimated drift speed')
figure(3);
hold off;
plot(x(clamp_time:end), current_hyp_t0(clamp_time:end)- s(clamp_time:end))
title('error (current t0 hypothesis - smoothed t0 hypothesis)')
