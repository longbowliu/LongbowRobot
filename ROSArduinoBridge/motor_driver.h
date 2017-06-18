/***********************************************************************
   Motor driver function definitions - by James Nugen and Chaoyang Liu
   ***********************************************************************/

  void initMotorController();
  void setMotorSpeed(int i, int spd);
  void setMotorSpeeds(int leftSpeed, int rightSpeed);
  #ifdef L298N_DUAL_HBRIDGE
    // motor one
//    #define ENA 5
    #define Right_motor_back 4
    #define Right_motor_go 5
    // motor two
//    #define ENB 6
    #define Left_motor_go 6
    #define Left_motor_back 7
  #endif
