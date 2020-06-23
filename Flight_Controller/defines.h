#ifndef DEFINESH
#define DEFINESH
/*
#if MOTOR_1_PIN < 8
#define MOTOR_1_HIGH PORTD |= 1<<MOTOR_1_PIN;
#define MOTOR_1_LOW PORTD &= ~(1<<MOTOR_1_PIN);
#define PIN_SETUP_1 DDRD |= 1<<MOTOR_1_PIN;
#endif

#if MOTOR_1_PIN > 7
#define MOTOR_1_HIGH PORTB |= 1<<(MOTOR_1_PIN-8);
#define MOTOR_1_LOW PORTB &= ~(1<<(MOTOR_1_PIN-8));
#define PIN_SETUP_1 DDRB |= 1<<(MOTOR_1_PIN-8);
#endif


#if MOTOR_2_PIN < 8
#define MOTOR_2_HIGH PORTD |= 1<<MOTOR_2_PIN;
#define MOTOR_2_LOW PORTD &= ~(1<<MOTOR_2_PIN);
#define PIN_SETUP_2 DDRD |= 1<<MOTOR_2_PIN;
#endif

#if MOTOR_2_PIN > 7
#define MOTOR_2_HIGH PORTB |= 1<<(MOTOR_2_PIN-8);
#define MOTOR_2_LOW PORTB &= ~(1<<(MOTOR_2_PIN-8));
#define PIN_SETUP_2 DDRB |= 1<<(MOTOR_2_PIN-8);
#endif



#if MOTOR_3_PIN < 8
#define MOTOR_3_HIGH PORTD |= 1<<MOTOR_3_PIN;
#define MOTOR_3_LOW PORTD &= ~(1<<MOTOR_3_PIN);
#define PIN_SETUP_3 DDRD |= 1<<MOTOR_3_PIN;
#endif

#if MOTOR_3_PIN > 7
#define MOTOR_3_HIGH PORTB |= 1<<(MOTOR_3_PIN-8);
#define MOTOR_3_LOW PORTB &= ~(1<<(MOTOR_3_PIN-8));
#define PIN_SETUP_3 DDRB |= 1<<(MOTOR_3_PIN-8);
#endif


#if MOTOR_4_PIN < 8
#define MOTOR_4_HIGH PORTD |= 1<<MOTOR_4_PIN;
#define MOTOR_4_LOW PORTD &= ~(1<<MOTOR_4_PIN);
#define PIN_SETUP_4 DDRD |= 1<<MOTOR_4_PIN;
#endif

#if MOTOR_4_PIN > 7
#define MOTOR_4_HIGH PORTB |= 1<<(MOTOR_4_PIN-8);
#define MOTOR_4_LOW PORTB &= ~(1<<(MOTOR_4_PIN-8));
#define PIN_SETUP_4 DDRB |= 1<<(MOTOR_4_PIN-8);
#endif


#define PIN_SETUP PIN_SETUP_1; PIN_SETUP_2;  PIN_SETUP_3; PIN_SETUP_4;*/

#define OUTPUT_AMOUNT (EXTRA_OUTPUT_AMOUNT+4)


#if PPM_INPUT_PIN < 8
#define INPUT_SETUP SREG |= B10000000; PCICR |= B00000100; PCMSK2 |= (1 << PPM_INPUT_PIN);
#define PPM_PIN_STATE ((PIND >> PPM_INPUT_PIN) & B00000001)
#define PPM_INTERRUPT PCINT2_vect
#endif

#if (PPM_INPUT_PIN > 7) && (PPM_INPUT_PIN < 14)
#define INPUT_SETUP SREG |= B10000000; PCICR |= B00000001; PCMSK0 |= (1<< (PPM_INPUT_PIN-8));
#define PPM_PIN_STATE ((PINB >> (PPM_INPUT_PIN-8)) & B00000001)
#define PPM_INTERRUPT PCINT0_vect
#endif

#if PPM_INPUT_PIN >19
#define INPUT_SETUP SREG |= B10000000;PCICR |= B00000010; PCMSK1 |= (1<< (PPM_INPUT_PIN - 20));
#define PPM_PIN_STATE ((PINC >> (PPM_INPUT_PIN-20)) & B00000001)
#define PPM_INTERRUPT PCINT1_vect
#endif

#define MAX_CORRECTION 500

#define GYRO_SCALE ((float)4000/GYRO_SCALE_FOR_AL) //Multiplying gyroData with this number will scale it so that 4000 represents 1000 degrees/second
#define GYRO_SCALE_FOR_AL 16384 //The value of gyroData when the copter is rotating 1000 degrees/second

#define GYRO_FACTOR_PITCH (1000.0/(float)maxAngularRatePitch)
#define GYRO_FACTOR_ROLL (1000.0/(float)maxAngularRateRoll)
#define GYRO_FACTOR_YAW (1000.0/(float)maxAngularRateYaw)

#define POS_SCALE 0.044444  //Multiplying pos[i] with this number will scale it so that 4000 represents 90 degreees position
#define POS_FACTOR (90.0/(float)AUTO_LEVEL_MAX_ANGLE) 

#define PID_VALUES_ACRO_START_ADDRESS 0
//#define PID_VALUES_ACRO_PW_ADDRESS 19
#define ACC_CALIB_START_ADDRESS 18
//#define ACC_CALIB_PW_ADDRESS 26
#define GYRO_CALIB_START_ADDRESS 24
//#define GYRO_CALIB_PW_ADDRESS 33
#define PID_VALUES_AL_START_ADDRESS 30
//#define PID_VALUES_AL_PW_ADDRESS 52
#define EEPROM_CHECK_ADDRESS 48
#define VALUES_IN_EEPROM_ADDRESS 49
#define MAX_RATES_START_ADDRESS 50
#define EXPO_START_ADDRESS 56


#define ACRO_PID 0
#define AL_PID 1
#define GYRO_CALIB 2
#define ACC_CALIB 3
#define MAX_RATES 4
#define EXPO 5

#if ARM_VALUE == 0
#define ARM_CHECK (savedInputVals[ARM_CHANNEL] < 1200)
#endif
#if ARM_VALUE == 1
#define ARM_CHECK ((savedInputVals[ARM_CHANNEL] >= 1200) && (savedInputVals[ARM_CHANNEL] <= 1800))
#endif
#if ARM_VALUE == 2
#define ARM_CHECK (savedInputVals[ARM_CHANNEL] > 1800)
#endif

#if BUZZER_ALARM_VALUE == 0
#define BUZZER_ALARM_CHECK (savedInputVals[BUZZER_ALARM_CHANNEL] < 1200)
#endif
#if BUZZER_ALARM_VALUE == 1
#define BUZZER_ALARM_CHECK ((savedInputVals[BUZZER_ALARM_CHANNEL] >= 1200) && (savedInputVals[BUZZER_ALARM_CHANNEL] <= 1800))
#endif
#if BUZZER_ALARM_VALUE == 2
#define BUZZER_ALARM_CHECK (savedInputVals[BUZZER_ALARM_CHANNEL] > 1800)
#endif

#if AUTO_LEVEL_ON_VALUE == 0
#define AUTO_LEVEL_ON_CHECK (savedInputVals[MODE_CHANGE_CHANNEL] < 1200)
#endif
#if AUTO_LEVEL_ON_VALUE == 1
#define AUTO_LEVEL_ON_CHECK ((savedInputVals[MODE_CHANGE_CHANNEL] >= 1200) && (savedInputVals[MODE_CHANGE_CHANNEL] <= 1800))
#endif
#if AUTO_LEVEL_ON_VALUE == 2
#define AUTO_LEVEL_ON_CHECK (savedInputVals[MODE_CHANGE_CHANNEL] > 1800)
#endif

#endif //DEFINESH
