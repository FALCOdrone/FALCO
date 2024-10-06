// TODO
// 1. GPS position convertion
// 2. Magnetometer yaw calculation
// 3. Multithreading for reading slow sensors

#include <Arduino.h>

#include "sensorFusion\EstimatorEKF.h"
#include "sensorFusion\baro.h"
#include "sensorFusion\gps.h"
#include "sensorFusion\imu.h"
#include "sensorFusion\mag.h"

#include "control\controller.h"
#include "control\motor.h"
#include "control\radio.h"

#include "common\pinDef.h"
#include "common\utils.h"

/*** PARAMETERS ***/
#define DEBUG 1
// #define CASCADE_PID
// #define RATE_PID

int GPSrate = 1;  // Rate at which GPS data is read in Hz

// Controller parameters (take note of defaults before modifying!):
float maxRoll = 30.0;   // Max roll angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxPitch = 30.0;  // Max pitch angle in degrees for angle mode (maximum ~70 degrees), deg/sec for rate mode
float maxYaw = 160.0;   // Max yaw rate in deg/sec

// Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing:
float B_madgwick = 0.04;  // Madgwick filter parameter
float B_accel = 0.14;     // Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
float B_gyro = 0.1;       // Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
#ifdef HAS_MAG
float B_mag = 1.0;  // Magnetometer LP filter parameter
#endif

/*** VARIABLES ***/
vec_t pos;
vec_t speed;
vec_t speedPrev;
vec_t accel;
vec_t accelPrev;
vec_t mag;
vec_t magPrev;
gps_t coord;
gps_t coordPrev;
quat_t quat;
attitude_t att;
attitude_t attPrev;
attitude_t passthru;
vec_t gyro;
vec_t gyroPrev;
bar_t bar;

vec_t accelWithOffset;
VectorXf accelWithOffset2(3);
VectorXf fixed_accel(3);
vec_t posGPS;
vec_t posGPS0;  // position (from gps) of starting point
gps_t coordGPS;
vec_t speedGPS;
float yawMag;
float lat0;         // latitude at starting point, used for projection (lat long -> x y)
float r = 6371000;  // earth radius (m)

// Desired values
vec_t desiredAccel;
quat_t desiredQuat;
attitude_t desiredAtt;
float desiredThrottle;

// Timing and debugging variables
float dt;
unsigned long currentTime, prevTime;
unsigned long blinkCounter, blinkDelay;
bool blinkAlternate;

// Radio communication
unsigned long radioIn[6] = {0};
unsigned long radioInPrev[6] = {0};

// Controller
PID_t PID;

#ifdef CASCADE_PID
PID_t PIDol;
#endif

// Motors
float motorsCmd[4] = {0};  // Throttle per motor 0->1

// Flight status
bool armedFly = false;

// Parameters for EKF
const int Nstate = 7;
VectorXf ini_state(Nstate);
VectorXf ini_stdDevs(Nstate);
VectorXf predict_state(Nstate);
MatrixXf R(3, 3);

// initialization of the constructor for estimation
QuadEstimatorEKF estimation;

void setup() {
    Serial.begin(115200);
    Serial.println("Initialization starting");
    initializeImu();
    initializeGPS();
    initializeMag();
    initializeBarometer();
    // initializeMotors();
    // initializeRadio();

    // setting initial values for estimation parameters/variables and initializing estimation obj
    ini_state.setZero();
    ini_stdDevs.setOnes();
    estimation.initialize(ini_state, ini_stdDevs);

    // setting initial values for estimation parameters/variables
    R << cos(PI / 4), sin(PI / 4), 0,
        -sin(PI / 4), cos(PI / 4), 0,
        0, 0, 1;
    lat0 = coordGPS.lat;
    posGPS0.x = r * coordGPS.lat;              // north
    posGPS0.y = r * coordGPS.lon * cos(lat0);  // east
    posGPS0.z = coordGPS.alt;

    // calibrateESCs(); //PROPS OFF. Uncomment this to calibrate your ESCs by setting throttle stick to max, powering on, and lowering throttle to zero after the beeps
    // Code will not proceed past here if this function is uncommented!

    // Indicate entering main loop with 3 quick blinks
    currentTime = micros();
    setupBlink(3, 160, 70);  // numBlinks, upTime (ms), downTime (ms)
    Serial.println("Initialization done");
}

void loop() {
    // Keep track of what time it is and how much time has elapsed since the last loop
    prevTime = currentTime;
    currentTime = micros();
    // dt = (currentTime - prevTime) / 1000000.0;

    loopBlink();  // Indicate we are in main loop with short blink every 1.5 seconds

    // Print data at 100hz (uncomment one at a time for troubleshooting) - SELECT ONE:
    // printRadioData();     //Prints radio pwm values (expected: 1000 to 2000)
    // printDesiredState();  //Prints desired vehicle state commanded in either degrees or deg/sec (expected: +/- maxAXIS for roll, pitch, yaw; 0 to 1 for throttle)
    // printGyroData();      //Prints filtered gyro data direct from IMU (expected: ~ -250 to 250, 0 at rest)
    // printAccelData();     //Prints filtered accelerometer data direct from IMU (expected: ~ -2 to 2; x,y 0 when level, z 1 when level)
    // printMagData();       //Prints filtered magnetometer data direct from IMU (expected: ~ -300 to 300)
    // printRollPitchYaw();  //Prints roll, pitch, and yaw angles in degrees from Madgwick filter (expected: degrees, 0 when level)
    // printPIDoutput();     //Prints computed stabilized PID variables from controller and desired setpoint (expected: ~ -1 to 1)
    // printMotorCommands(); //Prints the values being written to the motors (expected: 120 to 250)
    // printLoopRate();      //Prints the time between loops in microseconds (expected: microseconds between loop iterations)

    // Get arming status
    armedStatus();  // Check if the throttle cut is off and throttle is low.

    // Get vehicle state from IMU
    getAcceleration(&accel);                // Updates acceleration data (m/s^2)
    lpFilter(&accel, &accelPrev, B_accel);  // Low pass filter acceleration data (m/s^2)
    getGyro(&gyro);                         // Updates gyro data (deg/sec)
    lpFilter(&gyro, &gyroPrev, B_gyro);     // Low pass filter gyro data (deg/sec)
    // getAttitude(&quat, &att);               // Updates roll, pitch, and yaw angle estimates (degrees)
    // lpFilter(&att, &attPrev, B_madgwick);   // Low pass filter attitude data (degrees)

    // TODO: get position from gps data
    // How? Position from the starting point? coordinates?
    getGPS(&coordGPS, &speedGPS);  // Updates GPS data (degrees)
    getMag(&mag);                  // Updates magnetometer data (uT)
    getBarometer(&bar);            // Updates barometer data (hPa)

    // removing the angular offset
    accelWithOffset2(0) = accelWithOffset.x;
    accelWithOffset2(1) = accelWithOffset.y;
    accelWithOffset2(2) = accelWithOffset.z;

    fixed_accel = R * accelWithOffset2;  // body frame accelleration without offset
    yawMag = estimation.yawFromMag(mag, quat);

    // projection of gps coordinates to x, y, z
    posGPS.x = r * coordGPS.lat - posGPS0.x;              // north
    posGPS.y = r * coordGPS.lon * cos(lat0) - posGPS0.y;  // east
    posGPS.z = -coordGPS.alt + posGPS0.z;                 // down
    posGPS.dt = coordGPS.dt;

    // EKF estimation for attitude, speed and position
    estimation.kf_attitudeEstimation(fixed_accel, Vector3f(gyro.x, gyro.y, gyro.z), accelWithOffset.dt);  // quaternion attitude estimation
    estimation.getAttitude(&quat, &att);
    estimation.predict(fixed_accel, Vector3f(gyro.x, gyro.y, gyro.z), accelWithOffset.dt);  // prediction of the (x, y, z) position and velocity

    // compute the update from gps
    if (isGPSUpdated()) {
        estimation.updateFromGps(Vector3f(posGPS.x, posGPS.y, posGPS.z), Vector3f(speedGPS.x, speedGPS.y, speedGPS.z), posGPS.dt);
    }
    estimation.updateFromMag(yawMag, mag.dt);  // TODO: calculate yaw from magnetometer data
    estimation.updateFromBar(bar.pressure, bar.dt);

    estimation.getPosVel(&pos, &speed);

    // Compute desired state
    getDesState(radioIn, &desiredAtt, &desiredThrottle);  // Convert raw commands to normalized values based on saturated control limits

// PID Controller - SELECT ONE:
#if defined CASCADE_PID
    controlANGLE2(radioIn[0], desiredAtt, gyro, att, &attPrev, &PIDol, &PID);  // Stabilize on angle setpoint using cascaded method. Rate controller must be tuned well first!
#elif defined RATE_PID
    controlRATE(radioIn[0], desiredAtt, gyro, &gyroPrev, att, &PID);  // Stabilize on rate setpoint
#else
    controlANGLE(radioIn[0], desiredAtt, gyro, att, &PID);  // Stabilize on angle setpoint
#endif

    accelPrev = accel;  // Store previous acceleration for next loop iteration
    attPrev = att;      // Store previous attitude for next loop iteration
    gyroPrev = gyro;    // Store previous gyro for next loop iteration

    // Actuator mixing and scaling to PWM values
    controlMixer(desiredThrottle, motorsCmd, PID.out);  // Mixes PID outputs to scaled actuator commands -- custom mixing assignments done here

    // Throttle cut check
    throttleCut(radioIn[4], motorsCmd, &armedFly);  // Directly sets motor commands to low based on state of ch5

    // Command actuators
    driveMotors(motorsCmd);  // Sends command pulses to each motor pin using OneShot125 protocol

    // Get vehicle commands for next loop iteration
    getCommands(radioIn, radioInPrev);  // Pulls current available radio commands

    // Regulate loop rate
    feedGPS();
    loopRate(2000);  // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
}

void armedStatus() {
    // DESCRIPTION: Check if the throttle cut is off and the throttle input is low to prepare for flight.
    if ((radioIn[4] < 1500) && (radioIn[0] < 1050)) {
        armedFly = true;
    }
}

void getDesState(unsigned long radioIn[], attitude_t *desiredAtt, float *desiredThrottle) {
    // DESCRIPTION: Normalizes desired control values to appropriate values
    /*
     * Updates the desired state variables thro_des, roll_des, pitch_des, and yaw_des. These are computed by using the raw
     * RC pwm commands and scaling them to be within our limits defined in setup. thro_des stays within 0 to 1 range.
     * roll_des and pitch_des are scaled to be within max roll/pitch amount in either degrees (angle mode) or degrees/sec
     * (rate mode). yaw_des is scaled to be within max yaw in degrees/sec. Also creates roll_passthru, pitch_passthru, and
     * yaw_passthru variables, to be used in commanding motors/servos with direct unstabilized commands in controlMixer().
     */
    *desiredThrottle = (radioIn[0] - 1000.0) / 1000.0;  // Between 0 and 1
    desiredAtt->roll = (radioIn[1] - 1500.0) / 500.0;   // Between -1 and 1
    desiredAtt->pitch = (radioIn[2] - 1500.0) / 500.0;  // Between -1 and 1
    desiredAtt->yaw = (radioIn[3] - 1500.0) / 500.0;    // Between -1 and 1

    // Constrain within normalized bounds
    *desiredThrottle = constrain(*desiredThrottle, 0.0, 1.0);                // Between 0 and 1
    desiredAtt->roll = constrain(desiredAtt->roll, -1.0, 1.0) * maxRoll;     // Between -maxRoll and +maxRoll
    desiredAtt->pitch = constrain(desiredAtt->pitch, -1.0, 1.0) * maxPitch;  // Between -maxPitch and +maxPitch
    desiredAtt->yaw = constrain(desiredAtt->yaw, -1.0, 1.0) * maxYaw;        // Between -maxYaw and +maxYaw
}

void loopRate(int freq) {
    // DESCRIPTION: Regulate main loop rate to specified frequency in Hz
    /*
     * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
     * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until
     * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to
     * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
     * and remain above 2kHz, without needing to retune all of our filtering parameters.
     */
    float invFreq = 1.0 / freq * 1000000.0;
    unsigned long checker = micros();

    // Sit in loop until appropriate time has passed
    while (invFreq > (checker - currentTime)) {
        feedGPS();
        checker = micros();
    }
}

void loopBlink() {
    // DESCRIPTION: Blink LED on board to indicate main loop is running
    /*
     * It looks cool.
     */
    if (currentTime - blinkCounter > blinkDelay) {
        blinkCounter = micros();
        digitalWrite(LED_BUILTIN, blinkAlternate);  // Pin 13 is built in LED

        if (blinkAlternate == 1) {
            blinkAlternate = 0;
            blinkDelay = 100000;
        } else if (blinkAlternate == 0) {
            blinkAlternate = 1;
            blinkDelay = 2000000;
        }
    }
}

void calibrateESCs() {
    // DESCRIPTION: Used in void setup() to allow standard ESC calibration procedure with the radio to take place.
    /*
     *  Simulates the void loop(), but only for the purpose of providing throttle pass through to the motors, so that you can
     *  power up with throttle at full, let ESCs begin arming sequence, and lower throttle to zero. This function should only be
     *  uncommented when performing an ESC calibration.
     */
    while (true) {
        digitalWrite(LED_BUILTIN, HIGH);                      // LED on to indicate we are not in main loop
        getCommands(radioIn, radioInPrev);                    // Pulls current available radio commands
        getDesState(radioIn, &desiredAtt, &desiredThrottle);  // Convert raw commands to normalized values based on saturated control limits

        for (int i = 0; i < 4; i++) {
            motorsCmd[i] = radioIn[0];  // Pass through throttle to motors
        }

        throttleCut(radioIn[4], motorsCmd, &armedFly);  // Directly sets motor commands to low based on state of ch5

        driveMotors(motorsCmd);  // Sends command pulses to each motor pin using OneShot125 protocol

        // printRadioData(); //Radio pwm values (expected: 1000 to 2000)

        loopRate(2000);  // Do not exceed 2000Hz, all filter parameters tuned to 2000Hz by default
    }
}