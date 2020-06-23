#ifndef CONFIGH
#define CONFIGH



#define MOTOR_1_PIN 8  //2-13, or 20-23 for A0-A3      (A4 and A5 are the I2C pins)
#define MOTOR_2_PIN 3   //2-13, or 20-23 for A0-A3
#define MOTOR_3_PIN 4   //2-13, or 20-23 for A0-A3
#define MOTOR_4_PIN 7   //2-13, or 20-23 for A0-A3



/*
           /\
          /| \
           | 
           |
       1      2
        \    /
         \__/
         |  |           
         |__|
        /    \
       /      \
      4        3
 */

#define PPM_INPUT_PIN 9      //2-13, or 20-23 for A0-A3

#define EXTRA_OUTPUT_AMOUNT 0  //amount of extra outputs for servos and such (1-2ms signal that ESCs and servos understand)

#define EXTRA_OUTPUT_1_PIN 12
#define EXTRA_OUTPUT_2_PIN 13
#define EXTRA_OUTPUT_3_PIN 5
#define EXTRA_OUTPUT_4_PIN 6

//#define AUTO_LEVEL_MODE   //DOES NOT WORK

#define BUZZER

#ifdef BUZZER
  #define BUZZER_PIN 2
#endif

#define CHANNEL_AMOUNT 8    //amount of channels on your PPM receiver (4-10) (I only own 8-channel receivers, can't test the others, but it should work.)
                            // I have the TGY-ia6c (8 channels) and the TGY-ia6b, witch was supposed to have 6 channels, but suprisingly it had 8 channels. 
                            // I don't know is there any standard on the amount of channels, but I think 6 and 8 are quite common.
                           
#define PPM_ROLL      0   //The order of the channels on the PPM signal (0 is first, 1 is second and so on..)
#define PPM_PITCH     1
#define PPM_THROTTLE  2
#define PPM_YAW       3
#define PPM_AUX_1     4
#define PPM_AUX_2     5
#define PPM_AUX_3     6
#define PPM_AUX_4     7

#define EXPO_PITCH 0.5  //from 0.0 to 1.0, when 0.0 the channel acts linearly
#define EXPO_ROLL 1.0
#define EXPO_YAW 0.1

#define ARM_CHANNEL PPM_AUX_1
#define ARM_VALUE 2   // 0 = low ( <1200), 1 = middle (1200-1800), 2 = high ( >1800)

#ifdef BUZZER
#define BUZZER_ALARM_CHANNEL PPM_AUX_3   //uncomment if you don't want buzzer alarm on any aux switch
#define BUZZER_ALARM_VALUE 2  // 0 = low ( <1200), 1 = middle (1200-1800), 2 = high ( >1800)
#endif

#ifdef AUTO_LEVEL_MODE
#define MODE_CHANGE_CHANNEL PPM_AUX_2
#define AUTO_LEVEL_ON_VALUE 2 // 0 = low ( <1200), 1 = middle (1200-1800), 2 = high ( >1800)
#endif




 
///////////////////////////////////////////////
//sensor config
#define MPU6050
//TILT TO RIGHT(Y) MUST BE POSITIVE. TILT TO FRONT(X) MUST BE POSITIVE. CLOCKWISE ROTATION(Z) MUST BE POSITIVE

#define GYRO_X 1    //The values mean the index of the gyro values in an array, change them to correct sensor orientation. Values 0-2 must be used.
#define GYRO_Y 0
#define GYRO_Z 2

#define GYRO_X_INVERT 1   //  -1 to invert
#define GYRO_Y_INVERT 1
#define GYRO_Z_INVERT -1

#define ACC_X 1    //The values mean the index of the acc values in an array, change them to correct sensor orientation. Values 0-2 must be used.
#define ACC_Y 0
#define ACC_Z 2

#define ACC_X_INVERT -1   //  -1 to invert
#define ACC_Y_INVERT 1  
#define ACC_Z_INVERT 1  //Changing this doesn't affect anything yet. Invert the other axes to correct your sensor board orientation.

#define LOW_PASS_FILTER 3   //0-6, 0: 256 Hz,   1: 188Hz,  2: 98Hz,   3: 42 Hz,  4: 20 Hz,  5: 10 Hz, 6: 5Hz

////////////////////////////////////////////
#define BAUD_RATE 115200
#define USE_EEPROM //save PID values, acc/gyro calib values, max rates and expo values to unvolatile memory 

//////////////////////////////////

#define PID_Kp_PITCH_ACRO 0
#define PID_Ki_PITCH_ACRO 0
#define PID_Kd_PITCH_ACRO 15000

#define PID_Kp_ROLL_ACRO 0
#define PID_Ki_ROLL_ACRO 0
#define PID_Kd_ROLL_ACRO 15000

#define PID_Kp_PITCH_AL 0
#define PID_Ki_PITCH_AL 0
#define PID_Kd_PITCH_AL 50

#define PID_Kp_ROLL_AL 0
#define PID_Ki_ROLL_AL 0
#define PID_Kd_ROLL_AL 50

#define PID_Kp_YAW 0  //Yaw PID values are the same in both modes
#define PID_Ki_YAW 0
#define PID_Kd_YAW 0

#define MAX_ANGULAR_RATE_PITCH 250 //The maximum rate represented by full pitch
#define MAX_ANGULAR_RATE_ROLL 250 //The maximum rate represented by full roll
#define MAX_ANGULAR_RATE_YAW 180 //The maximum rate represented by full yaw

#define AUTO_LEVEL_MAX_ANGLE 45 //must be less than 90


////////////////////////////////
#define COMMAND_P_PITCH "p_pitch"
#define COMMAND_P_ROLL "p_roll"
#define COMMAND_P_YAW "p_yaw"
#define COMMAND_I_PITCH "i_pitch"
#define COMMAND_I_ROLL "i_roll"
#define COMMAND_I_YAW "i_yaw"
#define COMMAND_D_PITCH "d_pitch"
#define COMMAND_D_ROLL "d_roll"
#define COMMAND_D_YAW "d_yaw"
#define COMMAND_PRINT_PID "print_pid"
#define COMMAND_SAVE_PID "save_pid"
#define COMMAND_LOAD_DEFAULTS "default"
#define COMMAND_SAVE_ACC_CALIB "save_acc"
#define COMMAND_SAVE_GYRO_CALIB "save_gyro"
#define COMMAND_DELETE_ACC_CALIB "delete_acc" //delete data in eeprom
#define COMMAND_DELETE_GYRO_CALIB "delete_gyro"
#define COMMAND_DELETE_PID "delete_pid"
#define COMMAND_DO_ACC_CALIB "calib_acc"
#define COMMAND_DO_GYRO_CALIB "calib_gyro"
#define COMMAND_MAX_RATE_PITCH "r_pitch"
#define COMMAND_MAX_RATE_ROLL "r_roll"
#define COMMAND_MAX_RATE_YAW "r_yaw"
#define COMMAND_SAVE_MAX_RATES "save_rates"
#define COMMAND_DELETE_MAX_RATES "delete_rates"
#define COMMAND_PRINT_RATES "print_rates"
#define COMMAND_EXPO_PITCH "e_pitch"
#define COMMAND_EXPO_ROLL "e_roll"
#define COMMAND_EXPO_YAW "e_yaw"
#define COMMAND_PRINT_EXPO "print_expo"
#define COMMAND_SAVE_EXPO "save_expo"
#define COMMAND_DELETE_EXPO "delete_expo"
#define COMMAND_EXPO_FUNCTION "expo_funct" //print the function used for expo calculations and put it to some graphing calculator to visualize the expo curve. This command takes the expo as parameter. For example: expo_funct 0.5
#define COMMAND_BUZZER "buzz"


//////////////////////////////////


// transmitter value range. Can be measured with the calibration sketch
#define THROTTLE_MAX 2001
#define ROLL_MAX 2001
#define PITCH_MAX 2001
#define YAW_MAX 2001
#define THROTTLE_MIN 999
#define ROLL_MIN 996
#define PITCH_MIN 993
#define YAW_MIN 996
#define PITCH_MIDDLE 1499
#define ROLL_MIDDLE 1496
#define YAW_MIDDLE 1499

#endif // CONFIGH
