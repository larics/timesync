%% perform Holt-Winters filtering; needs alfa and beta set in workspace

% the median filter needs ~500 samples to stabilize itself, we clamp to it
% until this time passes
b(1:500)= -3e-7;
s(1:500) = med_filtered(1:500);
for i=501:size(s,1)
  s(i)=(1-alfa)*med_filtered(i)+alfa*(s(i-1)+b(i-1));
  b(i)=(1-beta)*(s(i)-s(i-1))+beta*b(i-1);
end

figure(1);
hold off;
plot(numbers(:,6));
plot(med_filtered);
hold on; 
plot(s);
legend('original signal', 'median filtered (input)', 'Holt-Winters filtered')
figure(2); 
hold off;
plot(b);
title('estimated drift speed')
figure(3);
hold off;
plot(numbers(:,6)- s)
title('error (current frame filtered timestamp - current clock)')
