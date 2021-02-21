# IMU Calibration
This article and repo describes how to calibrate an Inertial Measurement Unit (IMU), what is involved in this, and why we need to.

## Contents
- Overview
- Calibration Process
- Background theory
- Logging Data from IMU
- Finding device variance
- Detecting stationary moments
- Calibrating Accelerometer
- Calibrating Gyroscope


## Overview
Firstly, let's briefly go over what an IMU does specifically. These devices, sometimes large and mechanical, sometimes just a silicon-based microchip on a circuit board, have three main components:
- An Accelerometer, which measures linear acceleration (link)
- A Gyroscope, which measures angular (or rotational) velocity (link)
- A Magnetomoter, which measures magnetic heading - that is, direction of the magnetic field the device is in, which is usually just the Earth's magnetic field and therefore this refers to measuring magnetic North. 
Each of these components measures a **3-dimensional** vector - that is, acceleration/angular velocity/heading in 3 orthogonal axes (think in x, y and z). 

These data are used to know the current orientation of a device - eg. a smartphone - or how fast a device might be moving - eg. a smartphone, a quadcopter, etc. They often measure at a very high rate, eg. 1000 Hz or similar. We'll ignore magnetometer here, because the application I have in mind for this doesn't require magnetometer. 

Ok, that's briefly what they are and where they are used. Why do we need to calibrate them and what does this calibration mean?
Well, ideally, we wouldn't need to, but manufacturing error and just tiny imperfections in those processes, etc, mean that things aren't made perfectly. We model this as the IMU having two parameters per measurement: a **bias** and a **scale** error, that, if we model correctly and take into account, should mean that our measurements are as perfect as they're going to be. 

**Bias** is additive error. If you have a tape measure with a bias, it adds a tiny bit onto every measurement, like the 1 mark being 0.01 units off, and everything else is correct from there. So every measurement is 0.01 units off, and if you _subtracted_ this from every measurement, you'd be correct. 

**Scale** is multiplicative error. If you have a tape measurement with scale error, it would _multiply_ the measurement by some amount, like someone wrote the labels on the tape but then stretched the tape so 1 is _actually_ at 1.01, and 2 is _actually_ at 2.02 and so on. If you scaled every measurement _down_ by 1.01, then you would be correct. 

So instead of the accelerometer's measurement just being **a** = (**a_x**,**a_y**,**a_z**), we model this as 

**a_corrected** = (**s_x** * **a_x_raw** + **b_x**, etc)

Otherwise every measurement will be off. So that's why we need to calibrate, and that's what calibration means: **calibrating an IMU means figuring out the scale and bias and using those for future measurements**.

One final note before we get into precisely how this will happen: IMUs often are calibrated in the factories where they are made, if you pay enough for them. I'm using a MPU6050 from [Adafruit](https://www.adafruit.com/product/3886) which is nice and affordable, but I don't think it came calibrated. This is annoying because then I have no factory calibration to compare my process to. I could spend more money and get a calibrated one ... but then there's also the question of whether I can use it uncalibrated, and whether I can get their calibration to compare mine to. Another option is to simulate. 

## Calibration Process
I'm using the paper [A robust and easy to implement method for IMU calibration without external equipments](https://www.researchgate.net/publication/273383944_A_robust_and_easy_to_implement_method_for_IMU_calibration_without_external_equipments) by Tedaldi, Pretto and Menegatti, and think of this as a replication experiment: I'm going to implement what I believe is their method based on their paper and see how well it works.

Physically, the authors' method involves:
1. holding the IMU perfectly still for some amount of time an an **initialisation period**
2. rotating it to a different position
3. Holding it still for some smaller period of time, a **static measurement period**
4. repeating 2 and 3 until we have enough data
5. running the calibration algorithm on these collected data

The initialisation period allows us to find the inherent noise or inherent error of the device, to take this into account later when trying to detect when the IMU is static (yes, we can tell it "I held it still here" but human precision will only get so far. If the IMU records at 1000Hz, then a human defining the static moment may be less accurate than the algorithm). Each statuc measurement period, and therefore each different position, constitutes a different sampling of the acceleration data for us to use in the final parameter estimation. Each transition between different positions constitutes a sampling of the gyroscope data, for its parameters. Since acceleromter parameters are easier to estimate than gyroscope parameters, we do these first, and then with these we can go and estimate the gyro data. 

I'm going to go over the importance and use of the static periods and the transition periods, and how we estimate the parameters, but first lets go over the theory behind all of this so we know why we have arrived at this algorithm. 

## Background theory

## Logging Data from IMU
As stated above, I'm using a MPU6050 IMU from [Adafruit](https://www.adafruit.com/product/3886) and I have it connected to an Arduino, which then connects to this program. I'm using [Manash's ++ SerialPort library](https://github.com/manashmandal/SerialPort) to connect to the Arduino and read from the Serial Port. The [Arduino program](https://github.com/dmckinnon/imucal/blob/master/datastream/datastream.ino) is simple; it just reads from the IMU at the specified rate, and then sends these data minimally through the serial port. You can use any arduino for this as long as the baud rate works and the arduino has I2C (which I'm pretty sure is all of them). Also, I added some args so you can just log the data to a csv, and then load in a csv of (timestamp, accel vector, gyro vector) as input instead of live from the arduino. 

## Finding Device Variance

## Detecting Stationary Moments

## Calibrating Accelerometer

## Calibrating Gyroscope
