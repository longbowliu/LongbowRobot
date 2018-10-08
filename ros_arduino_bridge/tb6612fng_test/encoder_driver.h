/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
  
   
#ifdef ARDUINO_MY_COUNTER

  void initEncoders();
  void leftEncoderEvent();
  void rightEncoderEvent();
#endif


long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();


unsigned char  lEFT_LAST_DERECTION  = 1;
unsigned char  RIGHT_LAST_DERECTION  = 1;






