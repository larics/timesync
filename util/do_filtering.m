%% perform filtering with median and Holt-Winters
median_window_size = 3000;

sensor_framerate = 200;  % for adjusting X axis to seconds

alfa = 0.0004;
beta = 0.00004;

clamp_time = 120*200;
early_alfa_start = 0.05;
early_beta_start = 0.001;

use_med_filtering = true;
initial_b = -3.4e-7;


% median_window_size = 5000;
% 
% sensor_framerate = 20;  % for adjusting X axis to seconds
% 
% alfa = 0.001;
% beta = 0.0001;
% 
% clamp_time = 500;
% early_alfa = 0.1;
% early_beta = 0.0;
% 
% use_med_filtering = true;
% initial_b = -3e-7;
%%%%

n = size(numbers, 1);
s = zeros(n, 1);
b = s;

if use_med_filtering
    med_filtered = medfilt(median_window_size, numbers(:, 6));
else
    med_filtered = numbers(:, 6); % skip filtering
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
      
        used_alpha(i)=alfa;
        used_beta(i)=beta;
    end
end
%s(1:clamp_time) = med_filtered(1:clamp_time);

x = (1:n)/sensor_framerate;
figure(1);
hold off;
plot(x, numbers(:,6));
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
plot(x(clamp_time:end), numbers(clamp_time:end,6)- s(clamp_time:end))
title('error (current frame filtered timestamp - current clock)')
