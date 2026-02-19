% Test code for checking out the collected data
set(groot, 'defaultTextInterpreter', 'None');

file_name = './data_collection_2_harry/harry_falling_forward_2.csv';
T0 = readtable(file_name);

dataCols = setdiff(T0.Properties.VariableNames, {'timestamp','label'});
D0 = T0{:, dataCols};          % numeric data only
idx = [true; any(diff(D0) ~= 0, 2)];

T0 = T0(idx, :);
ts0 = datetime(T0.timestamp, 'InputFormat', 'yyyy-MM-dd''T''HH:mm:ss.SSS');
% time0  = seconds(T0.timestamp - T0.timestamp(1));
time0 = zeros(1, length(ts0));

for i = [2 : 1: length(ts0)]
    time0(i) = time0(i-1) + T0.MCU_TIME(i);
end

time0 = time0./1000;

figure_name = file_name + "; Sample size: " + size(time0, 1);


figure;
subplot(2,1,1);
%plot(time0, T0.AX);
hold on;
% plot(time0, T0.AY);
% plot(time0, T0.AZ);
plot(time0, T0.ASVM);
% legend("AX", "AY", "AZ", "ASVM");
title(figure_name);
hold off;

subplot(2,1,2);
% plot(time0, T0.GX);
hold on;
% plot(time0, T0.GY);
% plot(time0, T0.GZ);
plot(time0, T0.GSVM);
title(figure_name)
hold off;
% legend("GX", "GY", "GZ", "GSVM");
