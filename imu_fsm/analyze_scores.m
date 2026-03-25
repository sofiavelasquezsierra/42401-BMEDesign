% run find_thresholds.m first to get the stats in the proper arrays
% calculate scores based approach

BUF_SIZE = 200;
DEV_BUFFER_SIZE = 50;
CHECK_TRIGGER = 1.4;
IDLE_TRIGGER = 0.85;
ACCEL_DEV_THRESHOLD = 0.08;
GYRO_DEV_THRESHOLD = 17.1;
TILT_TRIGGER_ANGLE = 30;

% there really aren't that many stats being used to decide between
% fall/run/walk/jump_sit:
% 1. fall_tilt_check
% 2. stabilized_dev / walking_dev / running_dev
% 3. fall_impact

corresponding_states = ["DETECTED_FALL" "RUNNING" "WALKING" "WALKING" "JUMPING_OR_QUICK_SIT" "JUMPING_OR_QUICK_SIT" "JUMPING_OR_QUICK_SIT"];
tag_to_state_map = [tags' corresponding_states'];

% calculate score based on percent diff from the threshold values
n = length(group_labels);
