clear; clc;

tags = ["fall","run","walk","jump","sit","squat"];

folders_to_search = ["fsm_test_shanaya", "full_fsm_serial_100Hz_1_lilly",
                     "full_fsm_serial_test_1_lilly", "serial_data_test"];

% Constants
BUF_SIZE = 200;
DEV_BUFFER_SIZE = 50;
CHECK_TRIGGER = 1.4;
IDLE_TRIGGER = 0.85;
INIT_TILT_SIZE = 10;
FINAL_TILT_SIZE = 60;
ACCEL_DEV_THRESHOLD = 0.06;
GYRO_DEV_THRESHOLD = 16.3;
TILT_TRIGGER_ANGLE = 60;
TILT_TRIGGER_RATIO = 0.3;

set(groot, 'defaultTextInterpreter', 'none');        % titles, labels
set(groot, 'defaultAxesTickLabelInterpreter', 'none'); % tick labels
set(groot, 'defaultLegendInterpreter', 'none');     % legends

%% ------------------------------------------------------------
% Collect all CSV files first
%% ------------------------------------------------------------

all_files = [];

for f = 1:length(folders_to_search)

    files = dir(fullfile(folders_to_search(f),"*.csv"));

    for k = 1:length(files)

        files(k).folder = folders_to_search(f);

    end

    all_files = [all_files; files];

end

file_names = string({all_files.name});
file_paths = fullfile(string({all_files.folder}),string({all_files.name}));

%% ------------------------------------------------------------
% Preallocate result container
%% ------------------------------------------------------------

results = struct();

% Global arrays for feature-space visualization
all_ASVM_STD  = [];
all_GSVM_STD  = [];
all_AVG_ANGLE = [];
all_INIT_ANGLE = [];
all_MAX_ASVM  = [];
all_MIN_ASVM  = [];
all_POST_ASVM = [];

group_labels  = strings(0);

for t = 1:length(tags)

    tag = tags(t);

    % Find files matching tag
    tag_mask = contains(file_names,tag);

    tag_files = file_paths(tag_mask);

    n = length(tag_files);

    if n == 0
        continue
    end

    % Preallocate arrays (vectorized friendly)
    ASVM_STD   = nan(n,1);
    GSVM_STD   = nan(n,1);
    AVG_ANGLE  = nan(n,1);
    INIT_ANGLE = nan(n,1);
    MAX_ASVM   = nan(n,1);
    MIN_ASVM   = nan(n,1);
    POST_ASVM  = nan(n,1);

    for i = 1:n

        T0 = readtable(tag_files{i});

        % remove duplicate rows
        dataCols = T0.Properties.VariableNames( ...
            varfun(@isnumeric,T0,'OutputFormat','uniform'));

        D0 = T0{:,dataCols};
        idx = [true; any(diff(D0) ~= 0,2)];
        T0 = T0(idx,:);

        % ----------------------------------------------------
        % Detect event
        % ----------------------------------------------------

        idle_trigger = find(T0.ASVM <= IDLE_TRIGGER,1,'first');

        if isempty(idle_trigger)
            continue
        end
        
        end_idx = min(idle_trigger + BUF_SIZE, height(T0));

        % make sure to only search within the buffer size
        check_idx = find(T0.ASVM(idle_trigger:end_idx) >= CHECK_TRIGGER,1,'first');
        disp(check_idx)
        disp(tag_files{i})
        if isempty(check_idx)
            continue
        end

        check_trigger = idle_trigger + check_idx;


        fall_idx = idle_trigger:end_idx;

        fall_sample = T0.ASVM(fall_idx);
        fall_sample_g = T0.GSVM(fall_idx);

        fall_ax = T0.AX(fall_idx);
        fall_ay = T0.AY(fall_idx);
        fall_az = T0.AZ(fall_idx);

        % ----------------------------------------------------
        % Metrics
        % ----------------------------------------------------

        % std deviation (last DEV_BUFFER_SIZE samples)
        ASVM_STD(i) = std(fall_sample(end-DEV_BUFFER_SIZE + 1:end));
        GSVM_STD(i) = std(fall_sample_g(end-DEV_BUFFER_SIZE + 1:end));

        % tilt angle
        xz_dist = sqrt(fall_ax.^2 + fall_az.^2);
        horiz_angle = atan2(fall_ay,xz_dist) * (180/pi);

        ang_bound = check_idx + 50;
        if(ang_bound > 201)
            ang_bound = 201;
        end
        AVG_ANGLE(i) = mean(horiz_angle(check_idx: ang_bound));
        
        INIT_ANGLE(i) = mean(horiz_angle(1:check_idx));
    
        % min / max
        MAX_ASVM(i) = max(fall_sample);
        MIN_ASVM(i) = min(fall_sample);

        % post-event ASVM
        post_start = check_trigger + BUF_SIZE;

        if post_start < height(T0)

            post_end = min(post_start + BUF_SIZE, height(T0));

            POST_ASVM(i) = mean(T0.ASVM(post_start:post_end));

        end

    end

    % Remove NaNs from failed detections
    valid = ~isnan(ASVM_STD);
    
    % Append valid samples to global arrays
    all_ASVM_STD  = [all_ASVM_STD;  ASVM_STD(valid)];
    all_GSVM_STD  = [all_GSVM_STD;  GSVM_STD(valid)];
    all_AVG_ANGLE = [all_AVG_ANGLE; AVG_ANGLE(valid)];
    all_INIT_ANGLE = [all_INIT_ANGLE; INIT_ANGLE(valid)];
    all_MAX_ASVM  = [all_MAX_ASVM;  MAX_ASVM(valid)];
    all_MIN_ASVM  = [all_MIN_ASVM;  MIN_ASVM(valid)];
    all_POST_ASVM = [all_POST_ASVM; POST_ASVM(valid)];
    
    % Add labels for these samples
    group_labels  = [group_labels; repmat(tag,sum(valid),1)];

    results.(tag).ASVM_STD  = mean(ASVM_STD(valid));
    results.(tag).GSVM_STD  = mean(GSVM_STD(valid));
    results.(tag).AVG_ANGLE = mean(AVG_ANGLE(valid));
    results.(tag).MAX_ASVM  = mean(MAX_ASVM(valid));
    results.(tag).INIT_ANGLE = mean(INIT_ANGLE(valid));
    results.(tag).MIN_ASVM  = mean(MIN_ASVM(valid));
    results.(tag).POST_ASVM = mean(POST_ASVM(valid));
    results.(tag).N = sum(valid);

end

%% Display correlation results
figure

subplot(2,2,1)
gscatter(all_ASVM_STD, all_GSVM_STD, group_labels)
xlabel("ASVM STD")
ylabel("GSVM STD")
title("Motion intensity")
legend('NumColumns', 2);

subplot(2,2,2)
gscatter(all_ASVM_STD, all_AVG_ANGLE, group_labels)
xlabel("ASVM STD")
ylabel("Angle")
title("Motion vs posture")
legend('NumColumns', 2);

subplot(2,2,3)
gscatter(all_ASVM_STD, all_MAX_ASVM, group_labels)
xlabel("ASVM STD")
ylabel("Max ASVM")
title("Impact strength")
legend('NumColumns', 2);

subplot(2,2,4)
gscatter(all_POST_ASVM, all_AVG_ANGLE, group_labels)
xlabel("Post ASVM")
ylabel("Angle")
title("Post-event posture")
legend('NumColumns', 2);

%% Plot grouped bar chart with error bars
stats_names = ["ASVM_STD","GSVM_STD","AVG_ANGLE","MAX_ASVM","MIN_ASVM","POST_ASVM"];
num_stats = length(stats_names);
num_tags  = length(tags);

means = zeros(num_tags, num_stats);
stds  = zeros(num_tags, num_stats);

for t = 1:num_tags
    tag = tags(t);
    mask = (group_labels == tag);
    if sum(mask) == 0
        continue
    end
    
    means(t,1) = mean(all_ASVM_STD(mask));
    stds(t,1)  = std(all_ASVM_STD(mask));
    
    means(t,2) = mean(all_GSVM_STD(mask));
    stds(t,2)  = std(all_GSVM_STD(mask));
    
    means(t,3) = mean(all_AVG_ANGLE(mask));
    stds(t,3)  = std(all_AVG_ANGLE(mask));
    
    means(t,4) = mean(all_MAX_ASVM(mask));
    stds(t,4)  = std(all_MAX_ASVM(mask));
    
    means(t,5) = mean(all_MIN_ASVM(mask));
    stds(t,5)  = std(all_MIN_ASVM(mask));
    
    means(t,6) = mean(all_POST_ASVM(mask));
    stds(t,6)  = std(all_POST_ASVM(mask));

    means(t,7) = mean(all_INIT_ANGLE(mask));
    stds(t,7)  = std(all_INIT_ANGLE(mask));
end

%% Plot each stat in its own subplot (3x2)
figure;

for s = 1:6
    subplot(2,3,s);
    hold on;

    if s == 3
        % Angle subplot (subplot position 3)
        % 3 bars per tag: Init, Final, Diff
        angle_means = zeros(num_tags,3);  % columns: Init, Final, Diff
        angle_stds  = zeros(num_tags,3);
    
        for t = 1:num_tags
            tag = tags(t);
            mask = (group_labels == tag);
            if sum(mask) == 0
                continue
            end
            avg_init_vals  = all_INIT_ANGLE(mask);
            avg_final_vals = all_AVG_ANGLE(mask);
            diff_vals = abs(avg_final_vals) - abs(avg_init_vals);
       
            angle_means(t,1) = abs(mean(avg_init_vals));
            angle_means(t,2) = abs(mean(avg_final_vals));
            angle_means(t,3) = abs(mean(diff_vals));
            if(tag == "fall")
                fall_init = (avg_init_vals)'
                fall_final = (avg_final_vals)'
                fall_diff = (diff_vals)'
            end
            
            angle_stds(t,1) = std(avg_init_vals);
            angle_stds(t,2) = std(avg_final_vals);
            angle_stds(t,3) = std(abs(avg_final_vals - avg_init_vals));
        end
    
        hb = bar(1:num_tags, angle_means);  % grouped bars
        % Get the X positions of each bar group for error bars
        x = nan(num_tags,3);
        for k = 1:3
            x(:,k) = hb(k).XEndPoints;
        end
    
        hold on
        errorbar(x, angle_means, angle_stds, 'k.', 'LineWidth', 1);
        
        yline(TILT_TRIGGER_ANGLE, 'r', 'LineWidth', 1);
        ylabel('Tilt Angle (deg)');
        title('Angles per tag');
        legend('INIT','FINAL','DIFF','Location','southeast');
        set(gca, 'XTick', 1:num_tags, 'XTickLabel', tags);
        grid on;
        hold off;

    else
        % Other stats: 1 bar per tag
        hb = bar(1:num_tags, means(:,s));
        errorbar(1:num_tags, means(:,s), stds(:,s), 'k.', 'LineWidth', 1);
        set(gca, 'XTick', 1:num_tags, 'XTickLabel', tags);
        ylabel(stats_names(s));
        title(stats_names(s) + " per tag");
        
        % optional thresholds
        switch stats_names(s)
            case "ASVM_STD"
                yline(ACCEL_DEV_THRESHOLD, 'red', 'LineWidth', 1);
            case "GSVM_STD"
                yline(GYRO_DEV_THRESHOLD, 'red', 'LineWidth', 1);
            case "MAX_ASVM"
                yline(CHECK_TRIGGER, 'red', 'LineWidth', 1);
            case "MIN_ASVM"
                yline(IDLE_TRIGGER, 'red', 'LineWidth', 1);
        end
        grid on;
    end

    hold off;
end

sgtitle('IMU Statistics per Event Tag');  % super title
