/*********************************************************************
 *  ROSArduinoBridge
 
    ...

    All rights reserved.

 *********************************************************************/

#define USE_BASE      // Enable the base controller code
//#undef USE_BASE     // Disable the base controller code

/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
   //#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   //#define ROBOGAIA
   
   /* Encoders directly attached to Arduino board */
   #define ARDUINO_ENC_COUNTER

   /* L298 Motor driver*/
   #define L298_MOTOR_DRIVER
#endif

//#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h
#undef USE_SERVOS     // Disable use of PWM servos

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           50     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  // Loại bỏ AUTO_STOP_INTERVAL và lastMotorCommand
#endif

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;
long en_begin_left = 0;
long en_begin_right = 0;
long sai_so_left = 0;
long sai_so_right = 0;

/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}
float computePID(float error) {
    static float previous_error = 0;
    static float integral = 0;
    float kp = 0.1;  // Hệ số tỉ lệ
    float ki = 0.005;  // Hệ số tích phân
    float kd = 0.008;  // Hệ số vi phân

    // Tính toán các thành phần PID
    float proportional = kp * error;
    integral += error;  // Tích lũy sai số (tích phân)
    float derivative = error - previous_error;  // Tính toán vi phân
    previous_error = error;

    // Tổng hợp lại thành đầu ra PID
    return proportional + (ki * integral) + (kd * derivative);
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);
  
  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING:
    Serial.println(Ping(arg1));
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;
#endif
    
#ifdef USE_BASE
  case READ_ENCODERS:
    Serial.print(readEncoder(LEFT));
    Serial.print(" ");
    Serial.println(readEncoder(RIGHT));
    break;
   case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
  if (arg1 == 0 && arg2 == 0){
    moving = 0;
    setMotorSpeeds(0,0);
    resetPID();
    break;
  }
  if (arg1 != arg2) {
      moving = 1;
      setMotorSpeeds(arg1 * 5, arg2 * 5);
      en_begin_left = readEncoder(LEFT);
      en_begin_right = readEncoder(RIGHT);
      break;
  } else {
      moving = 1;
      // Tính sai số giữa encoder bên trái và bên phải
      sai_so_left = (readEncoder(LEFT) - en_begin_left) - (readEncoder(RIGHT) - en_begin_right);

      // Áp dụng PID để điều chỉnh sai số
      float pid_correction = computePID(sai_so_left);

      // Điều chỉnh tốc độ motor dựa trên kết quả của PID
      leftPID.TargetTicksPerFrame = arg1 - pid_correction;  // Sử dụng giá trị PID để điều chỉnh tốc độ motor trái
      rightPID.TargetTicksPerFrame = arg2;  // Motor phải vẫn giữ nguyên tốc độ mục tiêu
      break;
  }
  
  Serial.println("OK");
  break;

  // if (arg1 != arg2 || arg1 == 0 && arg2 == 0) {
  //     setMotorSpeeds(arg1, arg2);
  //     resetPID();
  //     moving = 0;
  //   }
  //   else {moving = 1;
  //     setMotorSpeeds(arg1, arg2);
  //   }
    
  //   leftPID.TargetTicksPerFrame = arg1;
  //   rightPID.TargetTicksPerFrame = arg2;
  //   Serial.println("OK"); 
  //   break;
    // if (arg1 != arg2 || arg1 == 0 && arg2 == 0) {
    //   setMotorSpeeds(arg1, arg2);
    //   resetPID();
    //   moving = 0;
    //   en_begin_left = readEncoder(LEFT);
    //   en_begin_right = readEncoder(RIGHT);
    // }
    // else {moving = 1;
    //   sai_so_left = readEncoder(LEFT) - en_begin_left - (readEncoder(RIGHT) - en_begin_right);
    //   setMotorSpeeds(arg1 - sai_so_left*5, arg2);
    // }
    // leftPID.TargetTicksPerFrame = arg1;
    // rightPID.TargetTicksPerFrame = arg2;
    // Serial.println("OK"); 
    // break;
  case MOTOR_RAW_PWM:
    // resetPID();
    // moving = 1; // Sneaky way to temporarily disable the PID
    // setMotorSpeeds(arg1 + 200, arg2);
    // Serial.println("OK");
    // break;
    // resetPID();
    // moving = 0; // Sneaky way to temporarily disable the PID
    // setMotorSpeeds(arg1, arg2);
    // if (arg1 == 0 && arg2 == 0) {
    //   setMotorSpeeds(0, 0);
    //   resetPID();
    //   moving = 0;
    // }
    // else moving = 1;
    // leftPID.TargetTicksPerFrame = arg1/4;
    // rightPID.TargetTicksPerFrame = arg2/4;
    // Serial.println("OK"); 
    // break;
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
}

/* Setup function--runs once at startup. */
void setup() {
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), ISR_leftEncoder, RISING);
    
    // Setup interrupt for the right encoder
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), ISR_rightEncoder, RISING);
  Serial.begin(BAUDRATE);

// Initialize the motor controller if used */
#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    //set as inputs
    DDRD &= ~(1<<LEFT_ENC_PIN_A);
    DDRD &= ~(1<<LEFT_ENC_PIN_B);
    DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    DDRC &= ~(1<<RIGHT_ENC_PIN_B);
    
    //enable pull up resistors
    PORTD |= (1<<LEFT_ENC_PIN_A);
    PORTD |= (1<<LEFT_ENC_PIN_B);
    PORTC |= (1<<RIGHT_ENC_PIN_A);
    PORTC |= (1<<RIGHT_ENC_PIN_B);
    
    // tell pin change mask to listen to left encoder pins
    PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
    // tell pin change mask to listen to right encoder pins
    PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
    
    // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
    PCICR |= (1 << PCIE1) | (1 << PCIE2);
  #endif
  initMotorController();
  resetPID();
#endif

/* Attach servos if used */
  #ifdef USE_SERVOS
    int i;
    for (i = 0; i < N_SERVOS; i++) {
      servos[i].initServo(
          servoPins[i],
          stepDelay[i],
          servoInitPosition[i]);
    }
  #endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval.
*/
void loop() {
  while (Serial.available() > 0) {
    
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
// If we are using base control, run a PID calculation at the appropriate intervals
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
#endif

// Sweep servos
#ifdef USE_SERVOS
  int i;
  for (i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
#endif
}
