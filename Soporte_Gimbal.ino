#include <SoftwareSerial.h>
#include <SPI.h>
#include <dogm_7036.h>
#include <LSM6.h>
#include <IMU.h>
#include <CF_lib.h>
#include <PID_v1.h>

double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
double Kp=3.0, Ki=1.0, Kd=0.0;
unsigned int Sampletime = 100;

PID ControlPID1(&Input1, &Output1, &Setpoint1, Kp, Ki, Kd, DIRECT);
PID ControlPID2(&Input2, &Output2, &Setpoint2, Kp, Ki, Kd, DIRECT);

LSM6 imu;
IMU imu_lib;
CF_lib cf_lib;
dogm_7036 DOG;

// Not all pins on the Leonardo support change interrupts
// only the following can be used: 8, 9, 10, 11. 
//SoftwareSerial bluetooth(10, 11); // RX, TX

int pitch_R, roll_R;

struct servoMoveMessage {
  uint8_t gimbal[2];
  uint8_t crc;  
};

void setup() {
  Serial.begin(115200);
  Serial.println("Entering setup()");

  bienvenida_LCD();
  
  initPID1();
  initPID2();

  pinMode(11, OUTPUT); //OC1A
  pinMode(12, OUTPUT); //OC1B
  configure_Timer1();
 
  imu_lib.begin(&imu);
  delay(100);
  imu_lib.getOffsetNoise();
  delay(100);
  
  int* accel_data = imu_lib.read_Acc();
  cf_lib.begin(accel_data);
 
  initBT();

  DOG.clear_display();   // Borrar display.         
  DOG.position(1,1);   //En la posicion 1 de la fila 1          
  DOG.string("Pitch: "); //Muestra en pantalla Pitch:
  DOG.position(1,2);   //Posicion 1 de la fila 2. 
  DOG.string("Roll: ");  //Muestra en pantalla Roll: 
    
  Serial.println("Exiting setup()");
}
 
void loop() {
  servoMoveMessage sms;
  
        int* accel_data = imu_lib.read_Acc();
        int* gyro_data = imu_lib.read_Gyro();
        int* noise_data = imu_lib.Get_Noise();
          
        float pitch_L = 90 + cf_lib.Compute_Pitch(accel_data, *(gyro_data+1), *(noise_data+4));
        float roll_L = 90 + cf_lib.Compute_Roll(accel_data, *(gyro_data), *(noise_data+3));
        Serial.print(F("pitch_L: ")); Serial.print(pitch_L); 
        Serial.print(F(" roll_L: ")); Serial.println(roll_L);
  while(Serial1.read() != 'S') {}
  while(Serial1.available() < sizeof(sms)) {};
  
  if(Serial1.available() >= sizeof(sms)) {
    Serial1.readBytes((byte*)&sms, sizeof(sms));   

    pitch_R = sms.gimbal[0]; 
    roll_R = sms.gimbal[1]; 
    uint8_t crc_R = sms.crc;
    uint8_t crc_L = XORchecksum8(sms.gimbal, sizeof(sms.gimbal));
    
    if(crc_R != crc_L) return;
    else {
      pitch_R = sms.gimbal[0];
      roll_R = sms.gimbal[1];
      if(pitch_R >= 0 && pitch_R <= 180 && roll_R >= 0 && roll_R <= 180) {
        Serial.print(F("pitch_R: ")); Serial.print(pitch_R); 
        Serial.print(F(" roll_R: ")); Serial.print(roll_R);
      
        //Calculo de ángulos locales
        int* accel_data = imu_lib.read_Acc();
        int* gyro_data = imu_lib.read_Gyro();
        int* noise_data = imu_lib.Get_Noise();
          
        float pitch_L = 90 + cf_lib.Compute_Pitch(accel_data, *(gyro_data+1), *(noise_data+4));
        float roll_L = 90 + cf_lib.Compute_Roll(accel_data, *(gyro_data), *(noise_data+3));
        //Serial.print(F("pitch_L: ")); Serial.print(pitch_L); 
        //Serial.print(F(" roll_L: ")); Serial.println(roll_L);
          
        //PID Compute 1
        Setpoint1 = pitch_R;
        Input1 = pitch_L;
        bool pid1_computed = ControlPID1.Compute();
         
        //PID Compute 2
        Setpoint2 = roll_R;
        Input2 = roll_L;
        bool pid2_computed = ControlPID2.Compute();
          
        OCR1A = Output1;
        OCR1B = Output2;
        Serial.print(F(" 1A: ")); Serial.print(OCR1A); Serial.print(F(" 1B: ")); Serial.println(OCR1B);
          
        if(pid1_computed && pid2_computed) {
          String s_pitch = String(cambio2(OCR1A));
          String s_roll = String(cambio2(OCR1B));
          const char* c_pitch = s_pitch.c_str();
          const char* c_roll = s_roll.c_str();
          
          DOG.position(8,1);   //En la posicion 1 de la fila 1          
          DOG.string("         ");      // Muestra el valor almacenado en c_num1.
          DOG.position(8,1);   //En la posicion 1 de la fila 1          
          DOG.string(c_pitch);      // Muestra el valor almacenado en c_num1.
          DOG.position(7,2);
          DOG.string("        ");     //Muestra el valor guardado en c_num2.
          DOG.position(7,2);
          DOG.string(c_roll);     //Muestra el valor guardado en c_num2. 
        }
      }
    }
  }
}

int cambio1(float x){
  return (x - (0)) * (2400 - 545) / (180 - (0)) + 545;
}

int cambio2(float x){
  return (x - (545)) * (180 - 90) / (2400 - (545)) + 0;
}

void initPID1() {
  Setpoint1 = 1500.0;
  Output1 = 0.0;
  ControlPID1.SetMode(AUTOMATIC);
  ControlPID1.SetOutputLimits(545, 2400); 
  ControlPID1.SetSampleTime(Sampletime); 
}

void initPID2() {
  Setpoint2 = 1500.0;
  Output2 = 0.0;
  ControlPID2.SetMode(AUTOMATIC);
  ControlPID2.SetOutputLimits(545, 2400); 
  ControlPID2.SetSampleTime(Sampletime); 
}

void configure_Timer1() {
  // Table 16-4. Waveform Generation Mode Bit Description
  // Mode WGM13 WGM12 WGM11 WGM10 Timer/Counter_Mode_of_Operation  TOP
  // 8    1     0     0     0     PWM, Phase and Frequency Correct ICR1
    
  // 16.11.1 TCCR1A – Timer/Counter1 Control Register A
  // COM1A1 COM1A0 COM1B1 COM1B0 - - WGM11 WGM10
  // COM1A1:0 and COM1B1:0 control the Output Compare pins (OC1A and OC1B) behavior.
  // Table 16-3. Compare Output Mode, Phase Correct and Phase and Frequency Correct PWM
  // COM1A1/COM1B1 COM1A0/COM1B0 Description
  //       1             0       Clear OC1A/OC1B on Compare Match when upcounting.
  //                             Set OC1A/OC1B on Compare Match when downcounting.
  TCCR1A |= _BV(COM1A1);
  TCCR1A &= ~(_BV(COM1A0));
  TCCR1A |= _BV(COM1B1);
  TCCR1A &= ~(_BV(COM1B0));
  TCCR1A &= (~(_BV(WGM11)) & ~(_BV(WGM10)));
  
  // 16.11.2 TCCR1B – Timer/Counter1 Control Register B
  // X X - WGM13 WGM12 CS12 CS11 CS10
  TCCR1B |= _BV(WGM13);
  TCCR1B &= ~(_BV(WGM12));
  
  // Table 16-5. Clock Select Bit Description
  // CS12 CS11 CS10 Description
  // 0    1    0    clk/8
  TCCR1B |= _BV(CS11);
  TCCR1B &= (~(_BV(CS12)) & ~(_BV(CS10)));

  // t_BIT_TCNT1: 1 / (16*10^6 / 8) = 0.5us
  ICR1 = 20000; // t_PWM: 2 * (20000 * 0.5us) = 20ms --> 50Hz
  OCR1A = 0; // 2 * (5000 * 0.5us) = 5ms
                // hay que multiplicar x2 porque es phase correct
  OCR1B = 0; // 2 * (7000 * 0.5us) = 7ms
                // hay que multiplicar x2 porque es phase correct
}

void bienvenida_LCD() {
  DOG.initialize(7,0,0,6,4,1,DOGM163);   //CSB=7, 0, 0 = use Hardware SPI, RS=6, 4 = RESET, 1 = 5V, EA DOGM163-A (=3 lines)
  DOG.displ_onoff(true);          //turn Display on
  DOG.cursor_onoff(false);         //turn Cursor blinking on
  DOG.contrast(0x0A);  // contraste de la pantalla.
  DOG.clear_display(); // borrar display

  DOG.position(1,1);           
  DOG.string("TFG: GIMBAL");      
  DOG.position(1,2);            
  DOG.string("Autor: J.Solis");      
  DOG.position(1,3);          
  DOG.string("Dir.: O.Casquero");     
}

void initBT() {
  Serial1.begin(115200); // Arrancamos modulo BT
  delay(100);
  Serial.println("Flushing serial interface incoming channel...");
  while(Serial1.available()>0)
    Serial1.read(); 
}

uint8_t XORchecksum8(const byte *data, size_t dataLength) {
  uint8_t value = 0;
  for(size_t i = 0; i < dataLength; i++) {
    value ^= (uint8_t)data[i];
  }
  
  return ~value;
}
