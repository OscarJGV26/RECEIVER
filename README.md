# RECEIVER
This library has a class named "RECEIVER" which acquires the data from the receiver in the Beaglebone Blue assuming is connected to the E4 pin.
To use it you must have installed the PRUs used by Ardupilot. Follow the "Install Debian" steps in (https://github.com/mirkix/BBBMINI/blob/master/doc/software/software.md) without step 16.

The begin function allows you to calibrate the receiver signals if you so choose and stores the calibration in 3 calibration files, namely "mid_points_calibration.txt","upper_limits_calibration.txt" and "lower_limits_calibration". The calibration routine takes you through the process. Additionally, this function checks the calibration files regardless of your choosing and decides whether to accept them or ask for a calibration. 

NOTE: It won't let you fly unless the calibration is proper.
  
The read function updates the channels array which you can use as input for other control functions.

An example is given in Receiver_Test.cpp. 
You can call this function as ./Receiver_Test 0.02.
