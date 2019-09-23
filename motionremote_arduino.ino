// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

//Communication via classic Bluetooth SPP
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

const int buttonPin = 17;
const int ledPin = 16;

int button, led;
int resettime;

int16_t ax, ay, az; //Output-Signal Accelerometer in LSB
int16_t gx, gy, gz; //Output-Signal Gyroscope in LSB
float ax_rel, ay_rel, az_rel; //Output-Signal Accelerometer in g

int ax_off, ay_off, az_off;
int gx_off, gy_off, gz_off;


float sens_a, sens_g; //Sensitivity in LSB/g bzw. LSB/deg bzw. LSB/°C
float temp, sens_temp, off_temp;

float pitch_a, roll_a; //Drehung um y, x laut Accelerometer in deg
float pitchrate_g1, rollrate_g1, yawrate_g1, pitchrate_g2, rollrate_g2, yawrate_g2; //Drehrate um x,y,z laut Gyroscope in deg/s
float pitch_g, roll_g, yaw_g; //Drehung um y, x, z laut Gyroscope in deg

float pitch_est, roll_est, yaw_est;

unsigned long t1, t2;
float delta_t;

char charbuff[20];
char charbuff_pitch[10], charbuff_roll[10], charbuff_yaw[10];
byte bytebuff[20];

int minidelay = 10;
int regulardelay = 50;

float value_a, value_g;

float rad2deg = 180 / 3.14159;

float mu = 0.1;
float w=0.5;

void setup() {

    Serial.begin(38400);
    
    SerialBT.begin("VRController"); //Bluetooth device name
    Wire.begin();

 
    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // get ranges
    Serial.print("accel range: ");
    Serial.println(accelgyro.getFullScaleAccelRange());
    switch(accelgyro.getFullScaleAccelRange()){
      case 0: sens_a = 16384.0; break;
      case 1: sens_a = 8192.0; break;
      case 2: sens_a = 4096.0; break;
      case 3: sens_a = 2048.0;
    }
    Serial.print("accel sensitivity: "); Serial.println(sens_a);
    
    Serial.print("accel range: ");
    Serial.println(accelgyro.getFullScaleGyroRange());
    switch(accelgyro.getFullScaleGyroRange()){
      case 0: sens_g = 131.0; break;
      case 1: sens_g = 65.5; break;
      case 2: sens_g = 32.8; break;
      case 3: sens_g = 16.4;
    }
    Serial.print("gyro sensitivity: "); Serial.println(sens_g);

    sens_temp = 340.0;
    off_temp = -521.0;
    pinMode(buttonPin,INPUT);
    pinMode(ledPin,OUTPUT); 

}

void loop() {
    button = digitalRead(buttonPin);
    
    temp = (accelgyro.getTemperature() - off_temp)/sens_temp + 35.0;
    
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    t2 = millis();

    //Calculate acceleration components in g
    ax_rel = (ax-ax_off)/sens_a;
    ay_rel = (ay-ay_off)/sens_a;
    az_rel = (az-az_off)/sens_a;

    //Calculate rotation rate components in deg/s
    rollrate_g2 = (gx-gx_off)/sens_g;
    pitchrate_g2 = (gy-gy_off)/sens_g;
    yawrate_g2 = (gz-gz_off)/sens_g;

    //Calculate vector value
    value_a = sqrt(ax_rel*ax_rel+ay_rel*ay_rel+az_rel*az_rel);
    if (value_a > 1.2 || value_a < 0.8){
      digitalWrite(ledPin,1);
    }
    else{
      digitalWrite(ledPin,0);
    }

    //Calculate pitch and roll angles from accelerator measurement
    roll_a = atan(ay_rel/(sqrt(ax_rel*ax_rel+az_rel*az_rel)))*rad2deg;
    if (az_rel >=0){
      pitch_a = atan2(-ax_rel,sqrt(az_rel*az_rel+mu*ay_rel*ay_rel))*rad2deg;
    }
    else{
      pitch_a = atan2(-ax_rel,-sqrt(az_rel*az_rel+mu*ay_rel*ay_rel))*rad2deg;
    }

    if (t1 == 0){
      pitch_est = pitch_a;
      roll_est = roll_a;
      yaw_est = 0.0;
    }
    else{
      delta_t = (t2 - t1)/1000.0;

      pitch_g = pitch_est + 0.5*(pitchrate_g1+pitchrate_g2)*delta_t;
      roll_g = roll_est + 0.5*(rollrate_g1+rollrate_g2)*delta_t;
      yaw_g = yaw_est + 0.5*(yawrate_g1+yawrate_g2)*delta_t;

      pitch_est = (1-w)*pitch_a+w*pitch_g;
      roll_est = (1-w)*roll_a+w*roll_g;
      if(yaw_g > 180.0){
        yaw_est = yaw_g-360.0;
      }
      else if(yaw_g <= -180.0){
        yaw_est = yaw_g + 360.0;
      }
      else{
        yaw_est = yaw_g;
      }
      
    }
        
    dtostrf(pitch_est,7,2,charbuff_pitch);
    dtostrf(roll_est,7,2,charbuff_roll);
    dtostrf(yaw_est,7,2,charbuff_yaw);


    for(int i=0; i<20; i++){
      if(i<7){
        charbuff[i]=charbuff_pitch[i];
      }
      else if(i>=7 && i<13){
        charbuff[i] = charbuff_roll[i-7];
      }
      else{
        charbuff[i]=charbuff_yaw[i-13];
      }
      bytebuff[i] = (byte)charbuff[i];

    }
    
    //Serial.println(charbuff);

    Serial.println("========== ACCELEROMETER ==========");
    Serial.print("X: "); Serial.print(ax_rel,2); Serial.print(" - Y: "); Serial.print(ay_rel,2); Serial.print(" - Z: "); Serial.println(az_rel,2);
    Serial.print("X: "); Serial.print(ax); Serial.print(" - Y: "); Serial.print(ay); Serial.print(" - Z: "); Serial.println(az);
    Serial.print("pitch: "); Serial.print(pitch_a,2); Serial.print(" - roll: "); Serial.print(roll_a,2); Serial.print(" - value: "); Serial.println(value_a,2);

    Serial.println("========== GYROSCOPE ==========");
    Serial.print("X: "); Serial.print(gx); Serial.print(" - Y: "); Serial.print(gy); Serial.print(" - Z: "); Serial.println(gz);
    Serial.print("pitchrate: "); Serial.print(pitchrate_g2,2); Serial.print(" - roll: "); Serial.print(rollrate_g2,2); Serial.print(" - yawrate: "); Serial.println(yawrate_g2,2);
    Serial.print("pitch: "); Serial.print(pitch_g,2); Serial.print(" - roll: "); Serial.print(roll_g,2); Serial.print(" - yaw: "); Serial.println(yaw_g,2);


    Serial.println("========== ESTIMATE ==========");
    Serial.print("delta_t: "); Serial.print(delta_t,3); Serial.print(" - t1: "); Serial.print(t1); Serial.print(" - t2: "); Serial.println(t2);
    Serial.print("pitch: "); Serial.print(pitch_est,2); Serial.print(" - roll: "); Serial.print(roll_est,2); Serial.print(" - yaw: "); Serial.println(yaw_est,2);

    Serial.print("temperature raw: "); Serial.print(accelgyro.getTemperature()); Serial.print(" - temperature in °C: "); Serial.println(temp,2);
     
    Serial.println("==============================");
    Serial.println(" ");

    
    SerialBT.write(bytebuff,20);
    
    delay(regulardelay);

    rollrate_g1 = rollrate_g2;
    pitchrate_g1 = pitchrate_g2;
    yawrate_g1 = yawrate_g2;
    t1 = t2;

    if (button == 1){
      resettime = 0;
      ax_off = 0;
      ay_off = 0;
      az_off = 0;
      gx_off = 0;
      gy_off = 0;
      gz_off = 0;
      
      Serial.println(">>>>>>>>>> ANGLE RESET <<<<<<<<<<");
      Serial.println("...initialized");
      while(button==1){
          button = digitalRead(buttonPin);
          resettime +=1;
          delay(minidelay);
      }
      Serial.print("...accumulating "); Serial.print(resettime); Serial.println(" iterations");
      t1 = 0;
      for(int i=0;i<resettime;i++){
        accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        ax_off += ax;
        ay_off += ay;
        az_off += az;
        gx_off += gx;
        gy_off += gy;
        gz_off += gz;
        
        delay(minidelay);
      }
      Serial.println("...done");
      digitalWrite(ledPin,1);
      delay(500);
      digitalWrite(ledPin,0);
      delay(500);
      digitalWrite(ledPin,1);
      delay(500);
      digitalWrite(ledPin,0);
      delay(500);
      digitalWrite(ledPin,1);
      delay(500);
      digitalWrite(ledPin,0);
      delay(500);

      ax_off = ax_off/resettime;
      ay_off = ay_off/resettime;
      az_off = az_off/resettime - sens_a;
      gx_off = gx_off/resettime;
      gy_off = gy_off/resettime;
      gz_off = gz_off/resettime;
      Serial.print("ax_off: "); Serial.print(ax_off); Serial.print(" - ay_off: "); Serial.print(ay_off); Serial.print(" - az_off: "); Serial.println(az_off);
      Serial.print("gx_off: "); Serial.print(gx_off); Serial.print(" - gy_off: "); Serial.print(gy_off); Serial.print(" - gz_off: "); Serial.println(gz_off);
      
      Serial.println(">>>>>>>>>> ANGLE RESET <<<<<<<<<<");
    }


}
