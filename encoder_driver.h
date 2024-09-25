/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
#ifdef ARDUINO_ENC_COUNTER
  // Sử dụng các chân thuộc PORTD và PORTC để tương thích với PCINT1_vect và PCINT2_vect
  #define LEFT_ENC_PIN_A 18  // PORTD, pin 21
  #define LEFT_ENC_PIN_B 19  // PORTD, pin 22
  
  #define RIGHT_ENC_PIN_A 20  // PORTC, pin 30
  #define RIGHT_ENC_PIN_B 21  // PORTC, pin 31
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
