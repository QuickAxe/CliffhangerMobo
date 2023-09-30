

//=============================--CLIFFHANGER--MOTHERBOARD--=======================================================//
//------------------------------------------V2-B--------------------------------------------------------------------//
//---------------------------------------16/09/23-----------------------------------------------------------------//



// Inputs :
// direction    ->  D2
// throttle     ->  D3
// MPU-6050 SDA ->  A4
// MPU-6050 SCL ->  A5
// Gain Pot     ->  A0

// Gain pot is used to set the gain value (duh!)

// Outputs:
// R_Rpwm -> D5
// R_Lpwm -> D6
// L_Rpwm -> D9
// L_Lpwm -> D10


#include <Wire.h>  // I2C library (for the MPU-6050 gyro /accelerometer)
#include <MPU6050.h>

MPU6050 mpu;


// Steering correction gain
byte mrscGain = 80;
float headingMultiplier = 2.0;                          // adjust until front wheels stay in parallel with the ground, if the car is swiveled around


// value up and down of midpoint to consider as dead zone
#define deadzone 5

#define R_Rpwm 5
#define R_Lpwm 6

#define L_Rpwm 9
#define L_Lpwm 10

#define GPot A0

// the angle of the x axis (roll) beyond which the bot is
// considered inverted
#define invertAngle 100

// timing variables to update  data at a regular interval
unsigned long now;
unsigned long rc_update;

// rc channels definition
const int channels = 2;  // specify the number of receiver channels
float RC_in[channels];   // an array to store the calibrated input from  receiver

int thr, dir;
int rVal, lVal;

// vectors to store gyro data
Vector normAccel;
Vector normGyro;

// to store roll value to check for inversion
int roll;


//float yaw_angle;
float yaw_rate;
float gyroFeedback;


void setup() {

    // making pins 9 and 10 as fast as 5 and 6
    // Pins D9 and D10 - 976 Hz
    TCCR1A = 0b00000001;  // 8bit
    TCCR1B = 0b00001011;  // x64 fast pwm

    pinMode(R_Lpwm, OUTPUT);
    pinMode(R_Rpwm, OUTPUT);
    pinMode(L_Lpwm, OUTPUT);
    pinMode(L_Rpwm, OUTPUT);


    setup_pwmRead();
    //Serial.begin(9600);


    // Initialize MPU6050
    //Serial.println("Initialize MPU6050");
    while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
        //Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
        delay(500);
    }


    // Calibrate gyroscope
    mpu.calibrateGyro();
}

void readMpu6050Data() {
    normGyro = mpu.readNormalizeGyro();
    normAccel = mpu.readNormalizeAccel();
}


// gyro based correction calculation
void mrsc() {

    int steeringAngle;
    long gyroFeedback;

    // Read sensor data
    readMpu6050Data();

    // calculate roll angle to find if bot is inverted or not
    roll = (atan2(normAccel.YAxis, normAccel.ZAxis) * 180.0) / M_PI;

    yaw_rate = normGyro.ZAxis * 0.001;
   // yaw_angle += yaw_rate;


    // reading gain value from pot
    mrscGain = map(analogRead(GPot), 0, 1023, 0, 100);


    // Compute steering compensation overlay
    int turnRateSetPoint = map(dir, -100, 100, -50, 50);         // turnRateSetPoint = steering angle (-100 to 100) = -50 to 50

    int steering = abs(turnRateSetPoint);                        // this value is required to compute the gain later on and is always positive

    int gain = map(steering, 0, 50, mrscGain, (mrscGain / 5));   // MRSC gain around center position is 5 times more!

    
    // if (steering < 5 && mrscGain > 85) {                      // Straight run @ high gain, "heading hold" mode -------------
    //     gyroFeedback = yaw_angle * headingMultiplier;         // degrees
    // } else {                                                  // cornering or low gain, correction depending on yaw rate in °/s --------------------------
     
    // removed "heading hold" mode due to gyro drift(very bad)
    
    gyroFeedback = yaw_rate * 50;                               // degrees/s * speed (always 50%)
       
       // yaw_angle = 0;                                        // reset yaw angle (heading direction)
    // }


    if (abs(roll) >= invertAngle)
        steeringAngle = turnRateSetPoint - (gyroFeedback * gain / 100);
    else
        steeringAngle = turnRateSetPoint + (gyroFeedback * gain / 100);

    steeringAngle = constrain(steeringAngle, -50, 50);         // range = -50 to 50

    dir = map(steeringAngle, -50, 50, -100, 100);

}


void move() {

    thr = RC_in[1] * 100;
    dir = RC_in[0] * 100;

    // Calculating gyro based corrections
    // and applying them to dir
    mrsc();

    // mixing the two channels 
    rVal = thr + dir;
    lVal = thr - dir;

    rVal = constrain(rVal, -100, 100);
    lVal = constrain(lVal, -100, 100);

    writeMotor();
}

void writeMotor() {
    if (rVal >= deadzone) {
        analogWrite(R_Lpwm, LOW);
        analogWrite(R_Rpwm, map(rVal, 0, 100, 0, 255));
        // Serial.print(" rVal = ");
        // Serial.print(map(rVal, 0, 100, 0, 255));
    } else if (rVal <= -deadzone) {
        analogWrite(R_Rpwm, LOW);
        analogWrite(R_Lpwm, map(abs(rVal), 0, 100, 0, 255));
        // Serial.print(" -rVal = ");
        // Serial.print(map(abs(rVal), 0, 100, 0, 255));
    } else {
        analogWrite(R_Lpwm, LOW);
        analogWrite(R_Rpwm, LOW);
        // Serial.print(" rVal = centre ");
    }

    if (lVal >= deadzone) {
        analogWrite(L_Lpwm, LOW);
        analogWrite(L_Rpwm, map(lVal, 0, 100, 0, 255));
        // Serial.print(" lVal = ");
        // Serial.print(map(lVal, 0, 100, 0, 255));

    } else if (lVal <= -deadzone) {
        analogWrite(L_Rpwm, LOW);
        analogWrite(L_Lpwm, map(abs(lVal), 0, 100, 0, 255));
        // Serial.print(" -lVal = ");
        // Serial.print(map(abs(lVal), 0, 100, 0, 255));
    } else {
        analogWrite(L_Lpwm, LOW);
        analogWrite(L_Rpwm, LOW);
        // Serial.print(" lVal = centre ");
    }
}

void loop() {

    now = millis();

    if (RC_avail() || now - rc_update > 25) {  // if RC data is available  or 25ms has passed since last update (adjust to be equal or greater than the frame  rate of receiver)

        rc_update = now;

        //print_RCpwm();  // uncommment to print raw  data from receiver to serial

        for (int i = 0; i < channels; i++) {  // run through each RC channel
            int CH = i + 1;

            RC_in[i] = RC_decode(CH);  // decode receiver channel and apply failsafe

            //print_decimal2percentage(RC_in[i]);  // uncomment to print calibrated  receiver input (+-100%) to serial
        }
        //Serial.println();  // uncomment when printing calibrated receiver input to serial.
    }

    move();

    // Serial.print(" Pitch = ");
    // Serial.print(pitch);
    // Serial.print(" Roll = ");
    // Serial.print(roll);

}
