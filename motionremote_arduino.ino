//=== INCLUDE LIBRARIES ===
#include "I2Cdev.h"
#include "MPU6050.h"
#include "BluetoothSerial.h"

//=== CHECK FUNCTIONALITIES ====
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//=== VARIABLES FOR BLUETOOTH COMMUNICATION ===
BluetoothSerial SerialBT;

char charbuff[41];
char charbuff_pitch[10], charbuff_roll[10], charbuff_yaw[10];
char charbuff_x[10], charbuff_y[10], charbuff_z[10];
byte bytebuff[41];


//=== VARIABLES FOR IMU CONTROLLING ===
MPU6050 accelgyro;

int16_t ax, ay, az; //Output-Signal Accelerometer in LSB
int16_t gx, gy, gz; //Output-Signal Gyroscope in LSB
float ax_rel, ay_rel, az_rel; //Output-Signal Accelerometer in g


float sens_a, sens_g; //Sensitivity in LSB/g bzw. LSB/deg bzw. LSB/°C
float temp, sens_temp, off_temp;

float value_a, value_g;


float pitch_a, roll_a, yaw_a = 0.0; //Drehung um y, x laut Accelerometer in deg
int ax_off, ay_off, az_off;
float pos_x, pos_y, pos_z;

int gx_off, gy_off, gz_off;
float pitchrate_g1, rollrate_g1, yawrate_g1; //Drehrate um x,y,z laut Gyroscope in deg/s
float pitchrate_g2, rollrate_g2, yawrate_g2; 
float pitchrate_gtemp, rollrate_gtemp, yawrate_gtemp; 
float pitch_g, roll_g, yaw_g; //Drehung um y, x, z laut Gyroscope in deg

float pitch_est, roll_est, yaw_est;



//=== VARIABLES FOR USER HANDLING ===

const int buttonPin = 17;
const int ledPin = 16;

int button, led;
int resettime;


//=== VARIABLES FOR CALCULATION ===
unsigned long t1, t2;
float delta_t;

int minidelay = 10;
int regulardelay = 50;

float rad2deg = 180 / 3.14159;

float mu = 0.1;
float w=0.8, w_roll, w_pitch, w_yaw;
float highpass_g = 0.5;

float acc_x, acc_y, acc_z;
float vel_x1, vel_y1, vel_z1;
float vel_x2, vel_y2, vel_z2;

float hist_x[3], hist_y[3], hist_z[3];
int zerocount_x, zerocount_y, zerocount_z;

float floatbuff1, floatbuff2, floatbuff3;

void setup() {
    //begin communication
    Serial.begin(38400);
    SerialBT.begin("VRController"); //Bluetooth device name
    Wire.begin();

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // get accelerometer range
    accelgyro.setFullScaleAccelRange(1);
    Serial.print("accel range: ");
    Serial.println(accelgyro.getFullScaleAccelRange());
    switch(accelgyro.getFullScaleAccelRange()){
      case 0: sens_a = 16384.0; break;
      case 1: sens_a = 8192.0; break;
      case 2: sens_a = 4096.0; break;
      case 3: sens_a = 2048.0;
    }
    Serial.print("accel sensitivity: "); Serial.println(sens_a);

    //get gyoroscope range
    Serial.print("accel range: ");
    Serial.println(accelgyro.getFullScaleGyroRange());
    switch(accelgyro.getFullScaleGyroRange()){
      case 0: sens_g = 131.0; break;
      case 1: sens_g = 65.5; break;
      case 2: sens_g = 32.8; break;
      case 3: sens_g = 16.4;
    }
    Serial.print("gyro sensitivity: "); Serial.println(sens_g);

    //calibrate temperature sensor
    sens_temp = 340.0;
    off_temp = -521.0;

    //set pins for user handling
    pinMode(buttonPin,INPUT);
    pinMode(ledPin,OUTPUT); 

}

void loop() {

    //initialize new loop iteration
    rollrate_g1 = rollrate_g2;
    pitchrate_g1 = pitchrate_g2;
    yawrate_g1 = yawrate_g2;
    t1 = t2;

    hist_x[0] = hist_x[1];
    hist_x[1] = hist_x[2];
    hist_x[2] = acc_x;

    hist_y[0] = hist_y[1];
    hist_y[1] = hist_y[2];
    hist_y[2] = acc_y;

    hist_z[0] = hist_z[1];
    hist_z[1] = hist_z[2];
    hist_z[2] = acc_z;
    
    //read user input
    button = digitalRead(buttonPin);
    if (button == 1){
      mpureset();
      }
    
    // read raw measurements
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    temp = (accelgyro.getTemperature() - off_temp)/sens_temp + 35.0;
    t2 = millis();

    //Calculate time step
    delta_t = (t2 - t1)/1000.0;
    
    //Calculate acceleration components in g
    ax_rel = (ax-ax_off)/sens_a;
    ay_rel = (ay-ay_off)/sens_a;
    az_rel = (az-az_off)/sens_a;

    //Calculate rotation rate components in deg/s
    rollrate_gtemp = (gx-gx_off)/sens_g;
    pitchrate_gtemp = (gy-gy_off)/sens_g;
    yawrate_gtemp = (gz-gz_off)/sens_g;

    rollrate_gtemp = highpass(rollrate_gtemp,highpass_g,0.0);
    pitchrate_gtemp = highpass(pitchrate_gtemp,highpass_g,0.0);
    yawrate_gtemp = highpass(yawrate_gtemp,highpass_g,0.0);

    rollrate_g2 = 1*rollrate_gtemp + sin(roll_est/rad2deg)*tan(pitch_est/rad2deg)*pitchrate_gtemp+cos(roll_est/rad2deg)*tan(pitch_est/rad2deg)*yawrate_gtemp;
    pitchrate_g2 = cos(roll_est/rad2deg)*pitchrate_gtemp-sin(roll_est/rad2deg)*yawrate_gtemp;
    yawrate_g2 = sin(roll_est/rad2deg)/cos(pitch_est/rad2deg)*pitchrate_gtemp+cos(roll_est/rad2deg)/cos(pitch_est/rad2deg)*yawrate_gtemp;

    rollrate_g2 = highpass(rollrate_g2,highpass_g,0.0);
    pitchrate_g2 = highpass(pitchrate_g2,highpass_g,0.0);
    yawrate_g2 = highpass(yawrate_g2,highpass_g,0.0);

    //Calculate acceleration vector value
    value_a = sqrt(ax_rel*ax_rel+ay_rel*ay_rel+az_rel*az_rel);
    if (value_a > 1.2 || value_a < 0.8){
      digitalWrite(ledPin,1);
    }
    else{
      digitalWrite(ledPin,0);
    }

    //Calculate pitch, roll and yaw angles from measurements
    floatbuff1=az_rel; floatbuff2=az_rel;
    roll_a = atan2(ay_rel,(az_rel/abs(floatbuff1))*(sqrt(ax_rel*ax_rel+az_rel*az_rel)))*rad2deg;
    pitch_a = atan2(-ax_rel,az_rel/abs(floatbuff2)*sqrt(az_rel*az_rel+mu*ay_rel*ay_rel))*rad2deg;

    if (t1 == 0){
      pitch_est = pitch_a;
      roll_est = roll_a;
      yaw_est = 0.0;
    }
    else{
      pitch_g = pitch_est + 0.5*(pitchrate_g1+pitchrate_g2)*delta_t;
      pitch_g = normalize(pitch_g,180.0,-180.0);
      
      roll_g = roll_est + 0.5*(rollrate_g1+rollrate_g2)*delta_t;
      roll_g = normalize(roll_g,180.0,-180.0);

      yaw_g = yaw_est + 0.5*(yawrate_g1+yawrate_g2)*delta_t;
      //Serial.println(yaw_g,2);
      yaw_g = normalize(yaw_g,180.0,-180.0);

      floatbuff1 = roll_a; floatbuff2 = roll_g;
      roll_a = lowpass(roll_a,175.0,roll_g/abs(floatbuff2)*180.0);
      if(roll_a*roll_g<0 && abs(floatbuff1)+abs(floatbuff2)>180.0){
        w_roll = 1.0;
      }
      else{
        w_roll = w;
      }
      
      floatbuff1 = pitch_a; floatbuff2 = pitch_g;
      pitch_a = lowpass(pitch_a,175.0,pitch_g/abs(floatbuff2)*180.0);
      if(pitch_a*pitch_g<0 && abs(floatbuff1)+abs(floatbuff2)>200.0){
        w_pitch = 1.0;
      }
      else{
        w_pitch = w;
      }

      if(yawrate_g2<3.0){
        w_yaw = 0.999;
      }
      else{
        w_yaw=1.0;
      }

      pitch_est = (1.0-w_pitch)*pitch_a+w_pitch*pitch_g;      
      roll_est = (1.0-w_roll)*roll_a+w_roll*roll_g;
      yaw_est = (1.0-w_yaw)*yaw_a+w_yaw*yaw_g;
      
    }

    //calculate acceleration without gravity
    acc_x = ax_rel-(-sin(pitch_est/rad2deg)*cos(roll_est/rad2deg));
    acc_y = ay_rel-(sin(roll_est/rad2deg));
    acc_z = az_rel-(cos(pitch_est/rad2deg)*cos(roll_est/rad2deg));

    acc_x = highpass(acc_x,0.01,0.0);
    acc_y = highpass(acc_y,0.002,0.0);
    acc_z = highpass(acc_z,0.01,0.0); 

    acc_x = lowpass(acc_x,4.0,0.0);
    acc_y = lowpass(acc_y,4.0,0.0);
    acc_z = lowpass(acc_z,4.0,0.0);     

    //integrate current velocity and position
    if(t1==0){
        vel_x2 = 0;
        vel_y2 = 0;
        vel_z2 = 0;
        
        vel_x1 = 0;
        vel_y1 = 0;
        vel_z1 = 0;

        pos_x = 0;
        pos_y = 0;
        pos_z = 0;

        hist_x[0] = 0;
        hist_x[1] = 0;
        hist_x[2] = 0;
        
        hist_y[0] = 0;
        hist_y[1] = 0;
        hist_y[2] = 0;

        hist_z[0] = 0;
        hist_z[1] = 0;
        hist_z[2] = 0;
      }
      else{
        if(acc_x == 0){
          vel_x2 = 0.5*vel_x1;
        }
        else{
          vel_x2 = vel_x1 + acc_x*delta_t;
        }
        
        if(acc_y == 0){
          vel_y2 = 0.5*vel_y1;
        }
        else{
          vel_y2 = vel_y1 + acc_y*delta_t;
        }

        zerocount_x = 0;
        for (int i=0; i<3; i++){
          if (hist_x[i] == 0){
            zerocount_x++;
          }
        }
        if(acc_x==0 && zerocount_x>=1){
          vel_x2 = 0.0;
        }
        else{
          vel_x2 = vel_x1 + acc_x*delta_t;
          vel_x2 = highpass(vel_x2,0.005,0.0);
          pos_x += 0.5*(vel_x2+vel_x1)*delta_t;
        }
        
        zerocount_y = 0;
        for (int i=0; i<3; i++){
          if (hist_y[i] == 0){
            zerocount_y++;
          }
        }
        if(acc_y==0 && zerocount_y>=1){
          vel_y2 = 0.0;
        }
        else{
          vel_y2 = vel_y1 + acc_y*delta_t;
          vel_y2 = highpass(vel_y2,0.005,0.0);
          pos_y += 0.5*(vel_y2+vel_y1)*delta_t;
        }

        zerocount_z = 0;
        for (int i=0; i<3; i++){
          if (hist_z[i] == 0){
            zerocount_z++;
          }
        }        
        if(acc_z==0 && zerocount_z>=1){
          vel_z2 = 0.0;
        }
        else{
          vel_z2 = vel_z1 + acc_z*delta_t;
          vel_z2 = highpass(vel_z2,0.005,0.0);
          pos_z += 0.5*(vel_z2+vel_z1)*delta_t;
        }

        if(vel_x2 == 0){
          pos_x = 0.995*pos_x;
        }

        if(vel_y2 == 0){
          pos_y = 0.995*pos_y;
        }
        if(vel_z2 == 0){
          pos_z = 0.995*pos_z;
        }
        
        vel_x1 = vel_x2;
        vel_y1 = vel_y2;
        vel_z1 = vel_z2;
      }
      pos_x = 0.0;
      pos_y = 0.0;

        
    //Send Values
    writeSerial();  
    writeBluetooth();
    delay(regulardelay);

}

float highpass(float var, float thresh, float set){
  floatbuff3 = abs(var);
  if(abs(floatbuff3) < thresh){
    return set;
  }
  else{
    return var;
  }
}

float lowpass(float var, float thresh, float set){
  floatbuff3 = abs(var);
  if(abs(floatbuff3) > thresh){
    return set;
  }
  else{
    return var;
  }
}

float normalize (float var, float upper, float lower){
  while (var>upper || var<lower){
    if (var>upper){
      var -= upper-lower;
    }
    else if(var<lower){
      var += upper-lower;
    }
  }
  

  return var;
}

void writeBluetooth(){
      dtostrf(pitch_est,7,2,charbuff_pitch);
      dtostrf(roll_est,7,2,charbuff_roll);
      dtostrf(yaw_est,7,2,charbuff_yaw);

      dtostrf(pos_x,7,2,charbuff_x);
      dtostrf(pos_y,7,2,charbuff_y);
      dtostrf(pos_z*100.0,7,2,charbuff_z);
  
      for(int i=0; i<41; i++){
      if(i<7){
        charbuff[i]=charbuff_pitch[i];
        //charbuff[i]='1';
      }
      else if(i>=7 && i<13){
        charbuff[i] = charbuff_roll[i-7];
        //charbuff[i]='2';
      }
      else if(i>=13 && i <20){
        charbuff[i]=charbuff_yaw[i-13];
        //charbuff[i]='3';
      }
      else if(i>=20 && i < 27){
        charbuff[i]=charbuff_x[i-20];
        //charbuff[i]='4';
      }
      else if(i>=27 && i < 34){
        charbuff[i]=charbuff_y[i-27];
        //charbuff[i]='5';
      }
      else if(i>=34){
        charbuff[i]=charbuff_z[i-34];
      }
  
        bytebuff[i] = (byte)charbuff[i];
  
      }

      SerialBT.write(bytebuff,41);

}

void writeSerial(){
  
    Serial.println("========== ACCELEROMETER ==========");
    Serial.print("X: "); Serial.print(ax_rel,2); Serial.print(" - Y: "); Serial.print(ay_rel,2); Serial.print(" - Z: "); Serial.println(az_rel,2);
    Serial.print("X: "); Serial.print(ax); Serial.print(" - Y: "); Serial.print(ay); Serial.print(" - Z: "); Serial.println(az);
    Serial.print("pitch: "); Serial.print(pitch_a,2); Serial.print(" - roll: "); Serial.print(roll_a,2); Serial.print(" - value: "); Serial.println(value_a,2);

    Serial.println("========== GYROSCOPE ==========");
    Serial.print("X: "); Serial.print(gx); Serial.print(" - Y: "); Serial.print(gy); Serial.print(" - Z: "); Serial.println(gz);
    Serial.print("pitchrate: "); Serial.print(pitchrate_g2,2); Serial.print(" - roll: "); Serial.print(rollrate_g2,2); Serial.print(" - yawrate: "); Serial.println(yawrate_g2,2);
    Serial.print("pitch: "); Serial.print(pitch_g,2); Serial.print(" - roll: "); Serial.print(roll_g,2); Serial.print(" - yaw: "); Serial.println(yaw_g,2);

    Serial.println("========== ANGLE ESTIMATE ==========");
    Serial.print("delta_t: "); Serial.print(delta_t,3); Serial.print(" - t1: "); Serial.print(t1); Serial.print(" - t2: "); Serial.println(t2);
    Serial.print("pitch: "); Serial.print(pitch_est,2); Serial.print(" - roll: "); Serial.print(roll_est,2); Serial.print(" - yaw: "); Serial.println(yaw_est,2);
    Serial.print("temperature raw: "); Serial.print(accelgyro.getTemperature()); Serial.print(" - temperature in °C: "); Serial.println(temp,2);

    Serial.println("========== POSITION ESTIMATE ==========");
    Serial.print("acc x: "); Serial.print(acc_x,3); Serial.print(" - acc y: "); Serial.print(acc_y,3); Serial.print(" - acc z: "); Serial.println(acc_z,3);
    Serial.print("vel x: "); Serial.print(vel_x2,3); Serial.print(" - vel y: "); Serial.print(vel_y2,3); Serial.print(" - vel z: "); Serial.println(vel_z2,3);
    Serial.print("pos x: "); Serial.print(pos_x,3); Serial.print(" - pos y: "); Serial.print(pos_y,3); Serial.print(" - pos z: "); Serial.println(charbuff_z);
   
    Serial.println("==============================");
    Serial.println(" ");
  
}

void mpureset(){
      resettime = 0;
      ax_off = 0;
      ay_off = 0;
      az_off = 0;
      gx_off = 0;
      gy_off = 0;
      gz_off = 0;
      
      Serial.println(">>>>>>>>>> ANGLE RESET <<<<<<<<<<");
      Serial.println("...initialized");
      digitalWrite(ledPin,1);
      while(button==1){
          button = digitalRead(buttonPin);
          resettime +=1;
          delay(minidelay);
      }
      digitalWrite(ledPin,0);
      Serial.print("...accumulating "); Serial.print(resettime); Serial.println(" iterations");
      t1 = 0;
      delay(1000);
      digitalWrite(ledPin,1);
      
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
      delay(200);
      digitalWrite(ledPin,0);
      delay(200);
      digitalWrite(ledPin,1);
      delay(200);
      digitalWrite(ledPin,0);
      delay(200);
      digitalWrite(ledPin,1);
      delay(200);
      digitalWrite(ledPin,0);
      delay(200);

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
