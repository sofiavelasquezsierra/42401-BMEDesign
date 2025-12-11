T = readtable('C:\Users\dlill\OneDrive\Desktop\VSC\ppg_calibration\harry_sensor_data1_reorg.csv');

IR_raw = T.col1;
Red_raw = T.col2;

data_time = 30; % how often each data point is coming in (ms)
num_samples = length(IR_raw);
t_start = 10;
t_end = 100;
t_raw = [0 : data_time/1000 : length(IR_raw)*data_time/1000 - data_time/1000]';
t = [9.99 : data_time/1000 : 99.99]';
data_start_idx = 9.99;
data_end_idx = 99.99;
IR = IR_raw(find(t_raw == data_start_idx) : find(t_raw == data_end_idx));
Red = Red_raw(find(t_raw == data_start_idx) : find(t_raw == data_end_idx));

% plotting the raw data
figure;
plot(t, IR);
hold on;
plot(t, Red);
hold off;