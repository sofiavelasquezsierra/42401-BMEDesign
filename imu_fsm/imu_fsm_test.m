% Test code for checking out the collected data

T0 = readtable('willa_fall_forward.csv');

dataCols = setdiff(T0.Properties.VariableNames, {'timestamp','label'});
D0 = T0{:, dataCols};          % numeric data only
idx = [true; any(diff(D0) ~= 0, 2)];

T0 = T0(idx, :);
ts0 = datetime(T0.timestamp, 'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss.SSS');
time0  = seconds(T0.timestamp - T0.timestamp(1));

figure;
plot(time0, T0.ASVM);
hold on;

hold off;
