time0_save_mask = (time0_save >= 2 & time0_save <= 6);
time1_save_mask = (time1_save >= 2 & time1_save <= 6);
time2_save_mask = (time2_save >= 2 & time2_save <= 6);
figure_name = "forward fall trials";

figure;
subplot(2,1,1);
%plot(time0, T0.AX);
% plot(time0, T0.AY);
% plot(time0, T0.AZ);
plot(time0_save(time0_save_mask), t0_save.ASVM(time0_save_mask));
hold on;
plot(time1_save(time1_save_mask), t1_save.ASVM(time1_save_mask));
plot(time2_save(time2_save_mask), t2_save.ASVM(time2_save_mask));

% legend("AX", "AY", "AZ", "ASVM");
title("Acceleration Signal Vector Magnitude", figure_name);
legend("trial 0", "trial 1", "trial 2");
xlabel("Time (s)");
ylabel("ASVM (G)");
hold off;

subplot(2,1,2);
% plot(time0, T0.GX);
hold on;
% plot(time0, T0.GY);
% plot(time0, T0.GZ);
plot(time0_save(time0_save_mask), t0_save.GSVM(time0_save_mask));
plot(time1_save(time1_save_mask), t1_save.GSVM(time1_save_mask));
plot(time2_save(time2_save_mask), t2_save.GSVM(time2_save_mask));
title("Gyroscope Signal Vector Magnitude", figure_name)
legend("trial 0", "trial 1", "trial 2");
xlabel("Time (s)");
ylabel("GSVM (rad/s)");
hold off;