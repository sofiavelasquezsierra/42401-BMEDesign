T = readtable('C:\Users\dlill\OneDrive\Desktop\VSC\42401-BMEDesign\ppg_calibration\ppg_calibration_data\andrew_hr_spo2_calibration_data0.csv');

t = [0 : 0.01 : length(T.ir_raw)*0.01 - 0.01];
IR = T.ir_raw;
Red = T.red_raw;
true_hr = T.true_hr;
true_spo2 = T.true_spo2;

% create data points from raw data
spo2_idx = zeros(1, length(IR));
spo2_idx(1) = 1; % track the first edge
for i = [2 : length(IR)]
    % detect edges
    if(true_spo2(i) ~= true_spo2(i-1))
        disp("found edge")
        spo2_idx(i) = 1;
    end
end
spo2_idx(end) = 1;

%% 
% raw data
figure;
subplot(2,1,1);
plot(t, IR);
hold on;
plot(t, Red);
ylabel("Reflected Light");
xlabel("Time (sec)");
legend("Raw IR", "Raw Red");
title("Raw Sensor Data");
hold off;

subplot(2,1,2);
plot(t, T.true_hr);
hold on;
plot(t, T.true_spo2);
title("Reference Sensor Values");
legend("Measured HR", "Measured SPO2");
ylabel("HR (bpm), SPO2 (%)");
xlabel("Time (sec)");
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
xline(0.2);
xline(1.2);
ylabel("Amplitude of Reflected Light");
xlabel("Frequency (Hz)");
title("Spectral Analysis of PPG Data");
legend("Red", "IR")
hold off;

%% 
Red_proc = bandpass(Red, [0.3 10], fs);
IR_proc = bandpass(IR, [0.3 10], fs);

Red_lp = lowpass(Red, 0.3, fs);
IR_lp = lowpass(IR, 0.3, fs);

spo2_sections = find(spo2_idx);
spo2_vals = true_spo2(spo2_sections(1:end - 1));
unique_vals = unique(spo2_vals);
ac_red = zeros(1, length(spo2_vals));
dc_red = zeros(1, length(spo2_vals));
ac_ir = zeros(1, length(spo2_vals));
dc_ir = zeros(1, length(spo2_vals));
for i = 1 : length(spo2_sections) - 1
    red_seg = Red_proc(spo2_sections(i) : spo2_sections(i+1));
    ir_seg = IR_proc(spo2_sections(i) : spo2_sections(i+1));

    red_seg_lp = Red_lp(spo2_sections(i) : spo2_sections(i+1));
    ir_seg_lp = IR_lp(spo2_sections(i) : spo2_sections(i+1));
 
    dc_red(i) = mean(red_seg_lp); % probably should use filtered data here but idk
    dc_ir(i) = mean(ir_seg_lp);


    ac_red(i) = abs((max(red_seg) - min(red_seg)) / 2);
    ac_ir(i) = abs((max(ir_seg) - min(ir_seg)) / 2);
end

ratio_red = (ac_red) ./ (dc_red);
ratio_ir = (ac_ir) ./ (dc_ir);

ratio = ratio_red ./ ratio_ir;

figure;
hold on;
scatter(ratio, spo2_vals);
hold on;
p = polyfit(ratio, spo2_vals, 1);
line_bf = p(1)*ratio + p(2);
title("Measured SPO2 vs. PPG AC/DC Ratio");
ylabel("Measured SPO2 (%)");
xlabel("AC/DC Ratio");

plot(ratio, line_bf);