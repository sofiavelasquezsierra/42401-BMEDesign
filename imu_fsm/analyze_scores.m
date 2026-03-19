% take a look at how far each event's stats are from the thresholds
% run find_thresholds.m first to get the stats in the proper arrays

% Constants - trimmed to only the ones we're using
BUF_SIZE = 200;
DEV_BUFFER_SIZE = 50;
CHECK_TRIGGER = 1.4;
IDLE_TRIGGER = 0.85;
ACCEL_DEV_THRESHOLD = 0.08;
GYRO_DEV_THRESHOLD = 17.1;
TILT_TRIGGER_ANGLE = 60;

% data is stored in the all_[STAT] arrays
% corresponding labels in group_labels array

% there really aren't that many stats being used to decide between
% fall/run/walk/jump_sit:
% 1. fall_tilt_check
% 2. stabilized_dev / walking_dev / running_dev
% 3. fall_impact

n = length(group_labels);

% get calculated state vs. expected state stats using reported value
for i = 1:n
    
end