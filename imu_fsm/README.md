Generally going off of this paper for the fall detection, but I'm extending it with additional checks for walking/running/jumping/quickly sitting down:
https://pmc.ncbi.nlm.nih.gov/articles/PMC12196599/#sec4-sensors-25-03632
- FSM = finite state machine, basically an algorithm where you move from state to state based on if the current data coming in from the sensors or other stats calculated from the data is above/below various thresholds

Key files:
- imu_fsm.ino -> program the board (later will be merged with Kristina's ppg code)
- imu_serial.py -> receive values over serial, for testing at 100Hz
- imu_fsm_display.m -> visualize data and calculate stats as needed to tweak algorithm thresholds

HOW TO TEST??
1. Plug in device to computer since the testing should be done over serial (we want 100Hz data rate, less than that kind of sucks, and Bluetooth testing extremely sucks).
2. Strap that device on. Technically (for IMU only) doesn't matter if you face the PPG sensor inwards or outwards, but it should be inwards. The important thing for the IMU to work is that you have it oriented horizontally, i.e. make sure the USB-C port hole is facing to the left or right side when you place the device on your chest.
3. Make sure device is programmed with imu_fsm.ino already (use Arduino IDE to do this)
4. Modify OUTPUT_FILE name on line 13 of imu_serial.py to be something relevant to what you're testing (needs to be a .csv file)
4. Run imu_serial.py: > python imu_serial.py
5. Wait until it says Connected. Recording... 
6. Do whatever action you're trying to test. If it's like running or speedwalking, do it for like 5-6 seconds at least so the FSM has a chance to run through the relevant states. Turning around while running/walking doesn't matter since we're primarily using the accelerometer data to make decisions here (so you don't need a huge room or hallway to test).
7. Ctrl+C to kill the python script and save all the data to the OUTPUT_FILE csv that you specified previously
8. Either look at the CSV file with your eyes or run the imu_fsm_display.m script in Matlab to see what you want to see