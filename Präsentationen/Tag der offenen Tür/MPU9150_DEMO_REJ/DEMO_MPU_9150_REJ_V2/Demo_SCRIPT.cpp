/* ----------- Setup --------------*/

 // Calibrate gyro and accelerometers, load biases in bias registers
 myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
 // Initialize device for active mode read of acclerometer, gyroscope, and
 // temperature
 myIMU.initMPU9250();

 // Get magnetometer calibration from AK8963 ROM
 myIMU.initAK8963(myIMU.factoryMagCalibration);

 // Get sensor resolutions, only need to do this once
 myIMU.getAres();
 myIMU.getGres();
 myIMU.getMres();

/* ----------- Loop --------------*/   

  myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
	// Now we'll calculate the accleration value into actual g's
  // This depends on scale being set
  myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
  myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
  myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

  myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

  // Calculate the gyro value into actual degrees per second
  // This depends on scale being set
  myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
  myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
  myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

  myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

  // Calculate the magnetometer values in milliGauss
  // Include factory calibration per data sheet and user environmental
  // corrections
  // Get actual magnetometer value, this depends on scale being set
  myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
             * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
  myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
             * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
  myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
             * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
	
  // Must be called before updating quaternions!
  myIMU.updateTime();
  
  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

	myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                * *(getQ()+3));
  myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                 * *(getQ()+2)));
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                * *(getQ()+3));
  myIMU.pitch *= RAD_TO_DEG;
  myIMU.yaw   *= RAD_TO_DEG;
	