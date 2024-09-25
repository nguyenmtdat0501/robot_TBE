/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */

#ifdef ARDUINO_ENC_COUNTER
  #include <util/atomic.h> // For the ATOMIC_BLOCK macro

  volatile long left_enc_pos = 0L;
  volatile long right_enc_pos = 0L;

  void setupEncoders() {
    // Setup interrupt for the left encoder
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), ISR_leftEncoder, RISING);
    
    // Setup interrupt for the right encoder
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), ISR_rightEncoder, RISING);
  }

  void ISR_leftEncoder() {
    int b = digitalRead(LEFT_ENC_PIN_B);
    if (b > 0) {
      left_enc_pos++;
    } else {
      left_enc_pos--;
    }
  }

  void ISR_rightEncoder() {
    int b = digitalRead(RIGHT_ENC_PIN_B);
    if (b > 0) {
      right_enc_pos++;
    } else {
      right_enc_pos--;
    }
  }

  /* Wrap the encoder reading function */
  long readEncoder(int i) {
    long pos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      if (i == LEFT) {
        pos = left_enc_pos;
      } else {
        pos = right_enc_pos;
      }
    }
    return pos;
  }

  /* Wrap the encoder reset function */
  void resetEncoder(int i) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      if (i == LEFT) {
        left_enc_pos = 0L;
      } else {
        right_enc_pos = 0L;
      }
    }
  }

  /* Wrap the encoder reset function for both encoders */
  void resetEncoders() {
    resetEncoder(LEFT);
    resetEncoder(RIGHT);
  }

#endif
