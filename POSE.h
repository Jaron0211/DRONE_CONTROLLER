void ang_control(float DES_PIT = 0, float DES_ROL = 0, float DES_YAW = 0, bool RESET = 0) {

  using namespace IMU;
  using namespace POSE;

  float PITCH_ANGLE_error = DES_PIT - PITCH * 15  ;
  float ROLL_ANGLE_error = DES_ROL  - ROLL * 15 ;

  //=============================================================================//

  float PITCH_error = gyro_pitch - PITCH_ANGLE_error;
  float ROLL_error = gyro_roll - ROLL_ANGLE_error;
  float YAW_error = gyro_yaw - (DES_YAW / 5);

  float time_pass = (millis() - PID_LOOP_TIMER);

  //PITCH
  PITCH_P_VAL = PITCH_error;

  PITCH_I_VAL += PITCH_error * POS_I * (time_pass / 1000);
  if (RESET) {
    PITCH_I_VAL = 0;
  }
  PITCH_I_VAL = constrain(PITCH_I_VAL, -POS_I_MAX, POS_I_MAX);

  data.PITCH_I_VAL = PITCH_I_VAL;

  PITCH_D_VAL = (PITCH_error - PITCH_error_pre) / (time_pass / 1000);
  PITCH_D_VAL = constrain(PITCH_D_VAL, -POS_D_MAX, POS_D_MAX);
  PITCH_error_pre = PITCH_error;

  data.PITCH_D_VAL = PITCH_D_VAL;

  PITCH_PID_OUTPUT = POS_P * PITCH_P_VAL + PITCH_I_VAL + POS_D * PITCH_D_VAL;
  data.PITCH_PID_OUTPUT = PITCH_PID_OUTPUT;

  //ROLL
  ROLL_P_VAL = ROLL_error;
  data.PITCH_P_VAL = ROLL_P_VAL;

  ROLL_I_VAL += ROLL_error * POS_I * (time_pass / 1000);
  if (RESET) {
    ROLL_I_VAL = 0;
  }
  ROLL_I_VAL = constrain(ROLL_I_VAL, -POS_I_MAX, POS_I_MAX);

  data.PITCH_I_VAL = ROLL_I_VAL;

  ROLL_D_VAL = (ROLL_error - ROLL_error_pre) / (time_pass / 1000);
  ROLL_D_VAL = constrain(ROLL_D_VAL, -POS_D_MAX, POS_D_MAX);
  ROLL_error_pre = ROLL_error;

  data.PITCH_D_VAL = ROLL_D_VAL;

  ROLL_PID_OUTPUT = POS_P * ROLL_P_VAL + ROLL_I_VAL + POS_D * ROLL_D_VAL;
  data.PITCH_PID_OUTPUT = ROLL_PID_OUTPUT;

  //YAW
  YAW_P_VAL = YAW_error;

  YAW_I_VAL += YAW_I * YAW_error * (time_pass / 1000);
  if (RESET) {
    YAW_I_VAL = 0;
  }
  if (YAW_I_VAL > POSE::YAW_I_MAX) {
    YAW_I_VAL = -POSE::YAW_I_MAX;
  } else if (YAW_I_VAL < -POSE::YAW_I_MAX) {
    YAW_I_VAL = -POSE::YAW_I_MAX;
  }
  YAW_I_VAL = constrain(YAW_I_VAL, -YAW_I_MAX, YAW_I_MAX);

  YAW_D_VAL = (YAW_error - YAW_error_pre) / (time_pass / 1000);
  YAW_D_VAL = constrain(YAW_D_VAL, -POSE::YAW_D_MAX, YAW_D_MAX);
  YAW_error_pre = YAW_error;

  YAW_PID_OUTPUT = YAW_P * YAW_P_VAL + YAW_I_VAL + YAW_D * YAW_D_VAL;

  PID_LOOP_TIMER = millis();

}

void pos_control(int input = 1500,float DES_PIT = 0, float DES_ROL = 0, float DES_YAW = 0, bool RESET = 0){
  using namespace IMU;
  using namespace POSE;
  using namespace BARO;

  float PITCH_ANGLE_error = DES_PIT - PITCH * 15  ;
  float ROLL_ANGLE_error = DES_ROL  - ROLL * 15 ;
  float HEI_error = ALT - PRE_ALT;

  //=============================================================================//

  float PITCH_error = gyro_pitch - PITCH_ANGLE_error;
  float ROLL_error = gyro_roll - ROLL_ANGLE_error;
  float YAW_error = gyro_yaw - (DES_YAW / 5);

  float time_pass = (millis() - PID_LOOP_TIMER);

  //PITCH
  PITCH_P_VAL = PITCH_error;

  PITCH_I_VAL += PITCH_error * POS_I * (time_pass / 1000);
  if (RESET) {
    PITCH_I_VAL = 0;
  }
  PITCH_I_VAL = constrain(PITCH_I_VAL, -POS_I_MAX, POS_I_MAX);

  data.PITCH_I_VAL = PITCH_I_VAL;

  PITCH_D_VAL = (PITCH_error - PITCH_error_pre) / (time_pass / 1000);
  PITCH_D_VAL = constrain(PITCH_D_VAL, -POS_D_MAX, POS_D_MAX);
  PITCH_error_pre = PITCH_error;

  data.PITCH_D_VAL = PITCH_D_VAL;

  PITCH_PID_OUTPUT = POS_P * PITCH_P_VAL + PITCH_I_VAL + POS_D * PITCH_D_VAL;
  data.PITCH_PID_OUTPUT = PITCH_PID_OUTPUT;

  //ROLL
  ROLL_P_VAL = ROLL_error;
  data.PITCH_P_VAL = ROLL_P_VAL;

  ROLL_I_VAL += ROLL_error * POS_I * (time_pass / 1000);
  if (RESET) {
    ROLL_I_VAL = 0;
  }
  ROLL_I_VAL = constrain(ROLL_I_VAL, -POS_I_MAX, POS_I_MAX);

  data.PITCH_I_VAL = ROLL_I_VAL;

  ROLL_D_VAL = (ROLL_error - ROLL_error_pre) / (time_pass / 1000);
  ROLL_D_VAL = constrain(ROLL_D_VAL, -POS_D_MAX, POS_D_MAX);
  ROLL_error_pre = ROLL_error;

  data.PITCH_D_VAL = ROLL_D_VAL;

  ROLL_PID_OUTPUT = POS_P * ROLL_P_VAL + ROLL_I_VAL + POS_D * ROLL_D_VAL;
  data.PITCH_PID_OUTPUT = ROLL_PID_OUTPUT;

  //YAW
  YAW_P_VAL = YAW_error;

  YAW_I_VAL += YAW_I * YAW_error * (time_pass / 1000);
  if (RESET) {
    YAW_I_VAL = 0;
  }
  if (YAW_I_VAL > POSE::YAW_I_MAX) {
    YAW_I_VAL = -POSE::YAW_I_MAX;
  } else if (YAW_I_VAL < -POSE::YAW_I_MAX) {
    YAW_I_VAL = -POSE::YAW_I_MAX;
  }
  YAW_I_VAL = constrain(YAW_I_VAL, -YAW_I_MAX, YAW_I_MAX);

  YAW_D_VAL = (YAW_error - YAW_error_pre) / (time_pass / 1000);
  YAW_D_VAL = constrain(YAW_D_VAL, -POSE::YAW_D_MAX, YAW_D_MAX);
  YAW_error_pre = YAW_error;

  YAW_PID_OUTPUT = YAW_P * YAW_P_VAL + YAW_I_VAL + YAW_D * YAW_D_VAL;
  
  //=============================================================================//
  
  ALT = 44307.6940*(1 - pow((PRES/1013.25),0.190284));

  int CONTROL_INPUT = input - 1500;
  CONTROL_INPUT = constrain(CONTROL_INPUT,-MAX_UP_RATE,MAX_UP_RATE);
  int DEL_PRES = CONTROL_INPUT * HEI_GAIN;

  HEI_P_VAL = HEI_error;
  HEI_I_VAL += HEI_error * HEI_I * (time_pass / 1000);
  if (RESET) {
    HEI_I_VAL = 0;
  }
  HEI_I_VAL = constrain(HEI_I_VAL, -HEI_I_MAX, HEI_I_MAX);

  data.HEI_I_VAL = HEI_I_VAL;

  HEI_D_VAL = (HEI_error - HEI_error_pre) / (time_pass / 1000);
  HEI_D_VAL = constrain(HEI_D_VAL, -HEI_D_MAX, HEI_D_MAX);
  HEI_error_pre = HEI_error;

  data.HEI_D_VAL = HEI_D_VAL;

  HEI_PID_OUTPUT = (HEI_P * HEI_P_VAL + HEI_I_VAL + HEI_D * HEI_D_VAL) + (HEI_GAIN * CONTROL_INPUT);
  data.HEI_PID_OUTPUT = HEI_PID_OUTPUT;
  
  PRE_ALT = ALT ;


  PID_LOOP_TIMER = millis();
}

void altitude_control(int input=1500,bool RESET = 0){
  using namespace BARO;
  using namespace POSE;
  ALT = 44307.6940*(1 - pow((PRES/1013.25),0.190284));

  int CONTROL_INPUT = input - 1500;
  CONTROL_INPUT = constrain(CONTROL_INPUT,-MAX_UP_RATE,MAX_UP_RATE);
  int DEL_PRES = CONTROL_INPUT * HEI_GAIN;
  int HEI_error = ALT - PRE_ALT;
  float time_pass = (millis() - PID_LOOP_TIMER);

  HEI_P_VAL = HEI_error;
  HEI_I_VAL += HEI_error * POS_I * (time_pass / 1000);
  if (RESET) {
    HEI_I_VAL = 0;
  }
  HEI_I_VAL = constrain(HEI_I_VAL, -POS_I_MAX, POS_I_MAX);

  data.HEI_I_VAL = HEI_I_VAL;

  HEI_D_VAL = (HEI_error - HEI_error_pre) / (time_pass / 1000);
  HEI_D_VAL = constrain(HEI_D_VAL, -POS_D_MAX, POS_D_MAX);
  HEI_error_pre = HEI_error;

  data.HEI_D_VAL = HEI_D_VAL;

  HEI_PID_OUTPUT = (POS_P * HEI_P_VAL + HEI_I_VAL + POS_D * HEI_D_VAL) + (HEI_GAIN * CONTROL_INPUT);
  data.HEI_PID_OUTPUT = HEI_PID_OUTPUT;
  
  PRE_ALT = ALT ;
  
  
}
