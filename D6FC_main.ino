#include <Wire.h>
#include <EEPROM.h>

#include "DEF.h"
#include "EEPROM.h"
#include "POSE.h"
#include "IO.h"
#include "IMU_SEN.h"
#include "COMMU.h"
#include "RC.h"

bool LED_STATE = 0;

void setup(){

  Serial.begin(115200);

  TWBR = 12;
  Wire.begin();

  //motor setup
  DDRC |= B11111111;
  PORTC |= B11111111;
  delay(1);
  PORTC &= B00000000;

  //led setup
  DDRB |= B11110000;
  PORTB |= B00010000;

  //calibrate gyro
  setup_IMU();
  Gyro_cal();
  cal_IMU_angle();
  PORTB |= B00100000;

  //baro setup
  setup_BARO();
  read_BARO();
  PORTB |= B01000000;

  //ISR SETUP
  if(RC::RC_MODE == 0){
    PCICR |= (1 << PCIE2);
    DDRK = B00000000;
    PCMSK2 = B11111111;
  }else{
    PCICR = B00000000;
    DDRK = B00000000;
    PCMSK2 = B00000000;
  }
  
  //read eeprom 
  READ_EEPROM();
  Serial.print(POS_P,4);
  Serial.print(",");
  Serial.print(POS_I,4);
  Serial.print(",");
  Serial.println(POS_D,4);

  PORTB |= B10000000;

  esc_output_timer = micros();
  ARM_TIMER = millis();
  PID_LOOP_TIMER = millis();
  BARO::BARO_READ_TIMER = millis();

  PORTB &= B00001111;

}

void loop(){
  if (micros() - esc_start_timer >= 20000) {
    PORTC |= B11111111;
    esc_start_timer = micros();
  }
  else {
    //esc signal output
    if (micros() - esc_start_timer >= M1_VAL) {
      PORTC &= B11111110;
    }
    if (micros() - esc_start_timer >= M2_VAL) {
      PORTC &= B11111101;
    }
    if (micros() - esc_start_timer >= M3_VAL) {
      PORTC &= B11111011;
    }
    if (micros() - esc_start_timer >= M4_VAL) {
      PORTC &= B11110111;
    }
    if (micros() - esc_start_timer >= M5_VAL) {
      PORTC &= B11101111;
    }
    if (micros() - esc_start_timer >= M6_VAL) {
      PORTC &= B11011111;
    }
    PORTB |= B10000000;
    //caculation
    if (micros() - esc_start_timer > 2500) {

      PORTB &= B01111111;
      //Serial
      Serial_RX();

      //sensor
      cal_IMU_angle();
      read_BARO();

      //INFO
      FAILSAFE();
      DEBUG_PRINT();
      //CH_PRINT();
      
      if (!ARM and !have_written) {
        WRITE_EEPROM();
        Serial.println("DATA SAVED");
        have_written = 1;
      }

      switch (MODE) {
      default:
        M1_VAL = MIN_SPEED;
        M2_VAL = MIN_SPEED;
        M3_VAL = MIN_SPEED;
        M4_VAL = MIN_SPEED;
        break;
      case 0:
        M1_VAL = MIN_SPEED;
        M2_VAL = MIN_SPEED;
        M3_VAL = MIN_SPEED;
        M4_VAL = MIN_SPEED;
        if (millis() - LED_TIMER > 500) {
          digitalWrite(LED1, LED_STATE);
          LED_STATE = !LED_STATE;
          LED_TIMER = millis();
        }
        break;
      case 1:
        STABLE();
        break;
      case 2:
        ESC_cali();
        break;
      }
    }
  }
}


ISR(PCINT2_vect) {
  using namespace RC;
  //every time interrupt active,update the timer
  channal_read_timer = micros();
  //ch1
  if (PINK & B00000001) {
    if (ch1_s == 0) {
      ch1_s = 1;
      ch1_timer = channal_read_timer;
    }
  }
  else if (ch1_s == 1) {
    ch1_s = 0;
    CH[0] = channal_read_timer - ch1_timer;
  }

  //ch2
  if (PINK & B00000010) {
    if (ch2_s == 0) {
      ch2_s = 1;
      ch2_timer = channal_read_timer;
    }
  }
  else if (ch2_s == 1) {
    ch2_s = 0;
    CH[1] = channal_read_timer - ch2_timer;
  }

  //ch3
  if (PINK & B00000100) {
    if (ch3_s == 0) {
      ch3_s = 1;
      ch3_timer = channal_read_timer;
    }
  }
  else if (ch3_s == 1) {
    ch3_s = 0;
    CH[2] = channal_read_timer - ch3_timer;
  }


  //ch4
  if (PINK & B00001000) {
    if (ch4_s == 0) {
      ch4_s = 1;
      ch4_timer = channal_read_timer;
    }
  }
  else if (ch4_s == 1) {
    ch4_s = 0;
    CH[3] = channal_read_timer - ch4_timer;
  }


  //ch5
  if (PINK & B00010000) {
    if (ch5_s == 0) {
      ch5_s = 1;
      ch5_timer = channal_read_timer;
    }
  }
  else if (ch5_s == 1) {
    ch5_s = 0;
    CH[4] = channal_read_timer - ch5_timer;
  }

  //ch6
  if (PINK & B00100000) {
    if (ch6_s == 0) {
      ch6_s = 1;
      ch6_timer = channal_read_timer;
    }
  }
  else if (ch6_s == 1) {
    ch6_s = 0;
    CH[5] = channal_read_timer - ch6_timer;
  }

  //ch7
  if (PINK & B01000000) {
    if (ch7_s == 0) {
      ch7_s = 1;
      ch7_timer = channal_read_timer;
    }
  }
  else if (ch7_s == 1) {
    ch7_s = 0;
    CH[6] = channal_read_timer - ch7_timer;
  }
  //ch8
  if (PINK & B10000000) {
    if (ch8_s == 0) {
      ch8_s = 1;
      ch8_timer = channal_read_timer;
    }
  }
  else if (ch8_s == 1) {
    ch8_s = 0;
    CH[7] = channal_read_timer - ch8_timer;
  }
 
  if ((channal_read_timer - ch5_timer < 900) or (channal_read_timer - ch5_timer > 2200)) {
    SIG_fail_safe = 1;
  }
}
