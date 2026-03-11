% Test code for checking out the collected data

T0 = readtable('./full_fsm_serial_test_1_lilly/lilly_test.csv');

dataCols = setdiff(T0.Properties.VariableNames, {'timestamp','label'});
D0 = T0{:, dataCols};          % numeric data only
idx = [true; any(diff(D0) ~= 0, 2)];

T0 = T0(idx, :);
ts0 = datetime(T0.timestamp, 'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss.SSS');
time0  = seconds(T0.timestamp - T0.timestamp(1));

figure;
subplot(2,1,1);
plot(time0, T0.AX);
hold on;
plot(time0, T0.AY);
plot(time0, T0.AZ);
plot(time0, T0.ASVM);
legend("AX", "AY", "AZ", "ASVM");
hold off;

subplot(2,1,2);
plot(time0, T0.GX);
hold on;
plot(time0, T0.GY);
plot(time0, T0.GZ);
plot(time0, T0.GSVM);
hold off;
legend("GX", "GY", "GZ", "GSVM");
