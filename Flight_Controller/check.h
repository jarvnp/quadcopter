#ifndef CHECKH
#define CHECKH

#if !(defined(__AVR_ATmega328P__) && F_CPU == 16000000)
  #error "Board not supported"
#endif 

#if !(((MOTOR_1_PIN >= 2)&&(MOTOR_1_PIN <= 13)) || ((MOTOR_1_PIN >= 20)&&(MOTOR_1_PIN <= 23)))
#error "MOTOR_1_PIN not supported"
#endif
#if !(((MOTOR_2_PIN >= 2)&&(MOTOR_2_PIN <= 13)) || ((MOTOR_2_PIN >= 20)&&(MOTOR_2_PIN <= 23)))
#error "MOTOR_2_PIN not supported"
#endif
#if !(((MOTOR_3_PIN >= 2)&&(MOTOR_3_PIN <= 13)) || ((MOTOR_3_PIN >= 20)&&(MOTOR_3_PIN <= 23)))
#error "MOTOR_3_PIN not supported"
#endif
#if !(((MOTOR_4_PIN >= 2)&&(MOTOR_4_PIN <= 13)) || ((MOTOR_4_PIN >= 20)&&(MOTOR_4_PIN <= 23)))
#error "MOTOR_4_PIN not supported"
#endif
#if !(((EXTRA_OUTPUT_1_PIN >= 2)&&(EXTRA_OUTPUT_1_PIN <= 13)) || ((EXTRA_OUTPUT_1_PIN >= 20)&&(EXTRA_OUTPUT_1_PIN <= 23)))
#error "EXTRA_OUTPUT_1_PIN not supported"
#endif
#if !(((EXTRA_OUTPUT_2_PIN >= 2)&&(EXTRA_OUTPUT_2_PIN <= 13)) || ((EXTRA_OUTPUT_2_PIN >= 20)&&(EXTRA_OUTPUT_2_PIN <= 23)))
#error "EXTRA_OUTPUT_2_PIN not supported"
#endif
#if !(((EXTRA_OUTPUT_3_PIN >= 2)&&(EXTRA_OUTPUT_3_PIN <= 13)) || ((EXTRA_OUTPUT_3_PIN >= 20)&&(EXTRA_OUTPUT_3_PIN <= 23)))
#error "EXTRA_OUTPUT_3_PIN not supported"
#endif
#if !(((EXTRA_OUTPUT_4_PIN >= 2)&&(EXTRA_OUTPUT_4_PIN <= 13)) || ((EXTRA_OUTPUT_4_PIN >= 20)&&(EXTRA_OUTPUT_4_PIN <= 23)))
#error "EXTRA_OUTPUT_4_PIN not supported"
#endif
#if !(((PPM_INPUT_PIN >= 2)&&(PPM_INPUT_PIN <= 13)) || ((PPM_INPUT_PIN >= 20)&&(PPM_INPUT_PIN <= 23)))
#error "PPM_INPUT_PIN not supported"
#endif
#if (defined(DETECT_PROGRAM_CRASHING) && !(defined(BUZZER)))
#error "buzzer needs to be defined for DETECT_PROGRAM_CRASHING"
#endif

#endif//CHECKH
