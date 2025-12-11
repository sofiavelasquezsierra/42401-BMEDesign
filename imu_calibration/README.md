You only need to run the imu_calibration routine once, or again if you notice there's 
some constant error after a while (unlikely unless the environment you're testing 
in is very different in terms of temperature or pressure on the sensor).
Just keep the IMU still and upload the code to the board.

When it's done the program will print out the six offsets for the gyroscope and 
accelerometer for use in other code (just subtract the offset from the raw sensor
readings). Takes about 400-800 samples to finish, be sure not to shake
the table that the IMU is on.

Example output:

******Calibration is done**********
gx offset: 1.10
gy offset: -2.45
gz offset: 0.96
ax offset: -0.06
ay offset: -0.02
az offset: 1.02
num samples taken: 529