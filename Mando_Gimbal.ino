#include <SPI.h>
#include <SoftwareSerial.h>
#include <LSM303.h>
#include <L3G.h>
#include <IMU_lib.h>
#include <CF_lib.h>

LSM303 accel;
L3G gyro;
IMU_lib imu_lib;
CF_lib cf_lib; 

String address = "0006666BB64A";//Direccion MAC de BT
int angulo = 90;              
int angulo2 = 90;             
int salto = 3;                 
int Eje_X = A1;
int Eje_Y = A2;
int z = 0;

volatile int flag = 0;

float pitch = 0;
float roll = 0;

struct servoMoveMessage {
   uint8_t gimbal[2];
   uint8_t crc;
};

void setup() {  
  Serial.begin(115200);
  Serial.println("setup()");

  imu_lib.beginAccel(&accel);
  imu_lib.beginGyro(&gyro);
  delay(100);
  imu_lib.getOffsetNoise();
  
  int* accel_data = imu_lib.Read_Accel();
  cf_lib.begin(accel_data);
  
  Init_BT(); 
  cx_BT2BT();
  
  pinMode(3, INPUT_PULLUP) ;
  attachInterrupt(1, boton, CHANGE);
  Serial.println(" Exiting setup()");
}
 
void loop() { 
  if(flag == 0) {
    int* accel_data = imu_lib.Read_Accel();
    int* gyro_data = imu_lib.Read_Gyro();
    int* noise_data = imu_lib.Get_Noise();

    roll = 90 + cf_lib.Compute_Roll(accel_data, *(gyro_data), *(noise_data+3));
    pitch = 90 + cf_lib.Compute_Pitch(accel_data, *(gyro_data+1), *(noise_data+4));
  } else {
    int p = analogRead(A1);
    int r = analogRead(A2);
    if(p < 400)                   
      pitch = pitch - salto ;   
      if(pitch < 0) pitch = 0;
    else if(p > 600)                  
      pitch = pitch + salto ;    
      if(pitch > 180) pitch = 180;
    if(r < 400)                    
      roll = roll - salto ;   
      if(roll < 0) roll = 0;
    else if(r > 600)                   
      roll = roll + salto ;    
      if(roll > 180) roll = 180;
  }

  Serial.print("pitch: "); Serial.print(pitch); 
  Serial.print(" roll: "); Serial.print(roll); 

  
  servoMoveMessage sms;
  sms.gimbal[0] = pitch;
  sms.gimbal[1] = roll;
  sms.crc = XORchecksum8(sms.gimbal, sizeof(sms.gimbal));
  Serial.print(" CRC: "); Serial.println(sms.crc);
   
  Serial1.print('S');
  Serial1.write((byte*)&sms, sizeof(sms));
}

int cambio1(float x){
  return ((x - (0)) * (2400 - 545) / (180 - (0)) + 545);
}

void boton() {
  delay(100);
  if(flag == 0)
    flag = 1;
  else
    flag = 0;
}

void Init_BT(){
  Serial1.begin(115200);// Comienza la comunicacion BT a 115200 baudios de velocidad.
  Serial1.print("$$$"); //BT entra en modo comando.
  delay(100);// Tiempo de espera.
  Serial1.println("U,9600,N"); // Modulo BT se comiucara a una velocidad de 9600.
  delay(100);
  Serial1.println("---");// Salir del modo comando.
  delay(100);
  while(Serial1.available()>0)
    Serial1.read(); 
    
  Serial1.begin(9600);
}

void cx_BT2BT() {
  Serial1.print("$$$"); 
  delay(100);
  Serial1.println("C," + address);  // connect. Conectar ambos modulos BT. La C sale en el data no pines.
  delay(3000);
  Serial1.println("---");
  delay(100);
  while(Serial1.available()>0)
    Serial1.read(); 
  Serial.println("Exit BT()");
}

uint8_t XORchecksum8(const byte *data, size_t dataLength) {
  uint8_t value = 0;
  for(size_t i = 0; i < dataLength; i++) {
    value ^= (uint8_t)data[i];
  }
  
  return ~value;
}

