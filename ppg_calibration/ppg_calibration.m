T = readtable('C:\Users\dlill\OneDrive\Desktop\VSC\42401-BMEDesign\ppg_calibration\ppg_calibration_data\iris_hr_spo2_calibration_data0.csv');

t = [0 : 0.01 : length(T.ir_raw)*0.01 - 0.01];
IR = T.ir_raw;
Red = T.red_raw;

% raw data
figure;
subplot(2,1,1);
plot(t, IR);
hold on;
plot(t, Red);
legend("IRraw", "Redraw");
hold off;

subplot(2,1,2);
plot(t, T.true_hr);
hold on;
plot(t, T.true_spo2);
legend("hr", "spo2");
hold off;

%% 
% do math
fs = 100; % sampling rate was 100hz
N = length(Red);
Red_proc = bandpass(Red - mean(Red), [0.3 5], fs);
IR_proc = bandpass(IR - mean(IR), [0.3 5], fs);
y = fft(Red_proc);
f = (0: floor(N/2)) * (fs/N);
P1 = abs(y)/length(y);
P1 = P1(1:floor(end/2) + 1);
figure;
idx = (f.*60 <= 400);
plot(f(idx), P1(idx), 'color', 'red');
hold on;
y_ir = fft(IR_proc);
P2 = abs(y_ir)/length(y_ir);
P2 = P2(1:floor(end/2) + 1);
plot(f(idx), P2(idx), 'color', 'black');
legend("Red", "IR")
xline(0.2);
xline(1.2);
hold off;

