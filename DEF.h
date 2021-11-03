
//HARDWARE DEFINETION
#define MPU6050 //IMU
#define HMC5883 // MAG
#define MS5611L // BARO

//sonar
#define FRONT_SONAR 0
#define BACK_SONAR 0
#define LEFT_SONAR 0
#define RIGHT_SONAR 0
#define TOP_SONAR 0
#define DOWN_SONAR 0
#define EX1_SONAR 0
#define EX2_SONAR 0

#define FRONT_SONAR_PIN 0
#define BACK_SONAR_PIN 0
#define LEFT_SONAR_PIN 0
#define RIGHT_SONAR_PIN 0
#define TOP_SONAR_PIN 0
#define DOWN_SONAR_PIN 0
#define EX1_SONAR_PIN 0
#define EX2_SONAR_PIN 0


int LED1 = 10; //channal status
int LED2 = 11; //ARM status
int LED3 = 12; 
int LED4 = 13;
unsigned long LED_TIMER = 0;


//POS CONTROL VALUE
#define MAX_ANGLE 45
#define MAX_YAW_ANGULAR 45

#define START_SPEED 1200
#define MIN_SPEED 1000
#define MAX_SPEED 2000


//===========================ISR_CH===========================//

unsigned long channal_read_timer;
unsigned long ch1_timer, ch2_timer, ch3_timer, ch4_timer, ch5_timer, ch6_timer,ch7_timer,ch8_timer;

boolean ch1_s, ch2_s, ch3_s, ch4_s, ch5_s, ch6_s, ch7_s, ch8_s;


//===========================SBUS===========================//

//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 6  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 20000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 29  //set PPM signal output pin on the arduino
//////////////////////////////////////////////////////////////////

int ppm[chanel_number];


//===========================FAILSAFE===========================//

namespace FS{
int FAIL_CODE = 0;

int SIG_fail_safe_counter = 0;
unsigned long SIG_fail_safe_timer = 0;
bool SIG_fail_safe = 0;
bool SIG_LOSE_FRAME = 0;

bool IMU_fail_safe = 0;
bool BARO_fail_safe = 0;
}
//===========================IMU===========================//
namespace IMU{
double IMU_ROTATION_DEGREE = 90;
double IMU_ROTATION_ANGLE = radians(IMU_ROTATION_DEGREE);

long gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector, acc_total_vector_level;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;

long gyro_pitch,gyro_roll,gyro_yaw;

float angle_pitch, angle_roll, angle_yaw;
float angle_roll_acc, angle_pitch_acc;
float PITCH, ROLL, YAW;

float pitch_cal, roll_cal, yaw_base ;
float pitch_base = 0;
float roll_base = 0;

int temperature;

bool set_gyro_angles;

bool IMU_request;
unsigned long IMU_request_timer;
unsigned long IMU_GET_TIMER;
long IMU_read_frequence;
}
//===========================BARO===========================//
namespace BARO{

  float TEMP ,PRES, ALT;
  unsigned long BARO_READ_TIMER;

  float PRES_DATA[6];
  float AVG_PRES,PRE_AVG_PRES;

  float HEI_P = 1;
  float HEI_I = 1;
  float HEI_D = 1;

  float HEI_GAIN = 1;

  float MAX_UP_RATE = 100;

  int HEI_I_MAX = 50;
  int HEI_D_MAX = 30;

  float HEI_P_VAL , HEI_I_VAL , HEI_D_VAL;
  float HEI_error_pre;
  long HEI_PID_OUTPUT;
  int PRE_ALT;
  
}

//===========================POSE===========================//
namespace POSE{
//PID
float CONTROL_GAIN = 5;

float POS_P = 1;
double POS_I = 1;
float POS_D = 1 ;

int POS_I_MAX = 100;
int POS_D_MAX = 100;

int GYRO_PID_MAX = 500;

float YAW_P = 3;
float YAW_I = 0;
float YAW_D = 1;
int YAW_I_MAX = 20;
int YAW_D_MAX = 20;

float HEIGHT_P = 0.1;
float HEIGHT_I = 0;
float HEIGHT_D = 0;


//define
float PITCH_P_VAL , PITCH_I_VAL , PITCH_D_VAL;
float ROLL_P_VAL , ROLL_I_VAL , ROLL_D_VAL;
float YAW_P_VAL , YAW_I_VAL , YAW_D_VAL;

float PITCH_error_pre, ROLL_error_pre, YAW_error_pre;

long PITCH_PID_OUTPUT, ROLL_PID_OUTPUT, YAW_PID_OUTPUT;

float FL_ROLL, FR_ROLL, BL_ROLL, BR_ROLL;
float FL_PITCH, FR_PITCH, BL_PITCH, BR_PITCH;
float FL_YAW, FR_YAW, BL_YAW, BR_YAW;
int FL_out, FR_out, BL_out, BR_out;

float ANG_PITCH_P_VAL , ANG_PITCH_I_VAL , ANG_PITCH_D_VAL;
float ANG_ROLL_P_VAL , ANG_ROLL_I_VAL , ANG_ROLL_D_VAL;
float ANG_YAW_P_VAL , ANG_YAW_I_VAL , ANG_YAW_D_VAL;

float ANG_PITCH_error_pre, ANG_ROLL_error_pre, ANG_YAW_error_pre;

long ANG_PITCH_PID_OUTPUT, ANG_ROLL_PID_OUTPUT, ANG_YAW_PID_OUTPUT;

unsigned long PID_LOOP_TIMER = 0;

}
//===========================RC===========================//
namespace RC{
int RC_MODE = 0;
  
int MODE = 2;

int CH1_MAX = 1932, CH1_MIN = 1108;
int CH2_MAX = 1932, CH2_MIN = 1108;
int CH3_MAX = 1932, CH3_MIN = 1108;
int CH4_MAX = 1932, CH4_MIN = 1108;
int CH5_MAX = 2000, CH5_MIN = 1000;
int CH6_MAX = 2000, CH6_MIN = 1000;

int CH[16];
}
//===========================Serial===========================//
namespace SER{
float PID_VAL[3] = {POSE::POS_P, POSE::POS_I, POSE::POS_D};

String DATA;

bool READ_START = 0;
bool REMOTE_CONTROLL = 0;
bool ANG_PID_ADJUST = 0;
bool GYR_PID_ADJUST = 0;
bool RX_END = 0;

bool debug = 0;
int DEBUGGING_DATA[30];

struct DEBUG_COLLECTION {
  
    float POS_P;
    float POS_I;
    float POS_D;
    
    float GYRO_PITCH;
    float GYRO_ROLL;
    float GYRO_YAW;
    float ANGLE_PITCH;
    float ANGLE_ROLL;
    
    float PITCH;
    float ROLL;
    float YAW;
    
    float PITCH_P_VAL;
    float PITCH_I_VAL;
    float PITCH_D_VAL;
    int PITCH_PID_OUTPUT;
    
    float ROLL_P_VAL;
    float ROLL_I_VAL;
    float ROLL_D_VAL;
    int ROLL_PID_OUTPUT;

    float HEI_P_VAL;
    float HEI_I_VAL;
    float HEI_D_VAL;
    int HEI_PID_OUTPUT;
    
    long IMU_read_frequence;

    float TEMP;
    float PRES;
    
}data;
}
//===========================OUTPUT===========================//
namespace MOT_IO{
int M1_VAL, M2_VAL, M3_VAL, M4_VAL, M5_VAL, M6_VAL;

unsigned long esc_output_timer;
unsigned long esc_start_timer;
unsigned long esc_ch_timer[6];

unsigned long ARM_TIMER;
boolean ARM;
}
//===========================sys_data===========================//

struct EEPROM_STRUCT {

  float CONTROL_GAIN;

  float POS_P;
  double POS_I;
  float POS_D;

  int POS_I_MAX;
  int POS_D_MAX;
  int GYRO_PID_MAX;

  float YAW_P;
  float YAW_I;
  float YAW_D;
  int YAW_I_MAX;
  int YAW_D_MAX;

  float HEIGHT_P;
  float HEIGHT_I;
  float HEIGHT_D;

  double IMU_ROTATION_DEGREE;
}; EEPROM_STRUCT memory;

bool have_written = 1;

//===========================Avoidance===========================//
namespace avoidance{
  int SONAR_VAL[8];
}
