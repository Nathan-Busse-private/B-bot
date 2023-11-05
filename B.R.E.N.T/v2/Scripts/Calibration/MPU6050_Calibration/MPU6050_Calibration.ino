// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int gyro_deadzone=1;     //Gyro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
MPU6050 accelgyro(0x68); // default <-- use for AD0 high

int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

int mpuConnection;

///////////////////////////////////   SETUP   ////////////////////////////////////
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  // COMMENT NEXT LINE IF YOU ARE USING ARDUINO DUE
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Leonardo measured 250kHz.

  // initialize serial communication
  Serial.begin(9600);

  // initialize MPU-6050
  accelgyro.initialize();

  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  
  Serial.println("Place the MPU-6050 breakout board in a flat or horizontal position, with SMD components facing up.\n");
  Serial.println(F("Type in any character and press Enter/Send to start MPU-6050 calibration..."));
  
  while (Serial.available() == 0){ } //wait for character to be entered               
  while (Serial.available() && Serial.read()); // empty buffer again

  // start message
  Serial.println("\n--------------------------------------------------------------");
  Serial.println("\nStarting MPU-6050 Calibration Sketch.");
  delay(1000);
  Serial.println("\nDon't touch the MPU-6050 until you see a \"FINISHED!\" message.");
  delay(2000);
  
  // verify connection
  //Serial.println(accelgyro.testConnection() ? "MPU-6050 connection SUCCESSFUL!" : "MPU-6050 connection FAILED.");
  //delay(1000);
  Serial.println("\n--------------------------------------------------------------");
  Serial.println("\nVerifying MPU-6050 connection...");
  delay(500);

  mpuConnection = accelgyro.testConnection();

  if (mpuConnection == 0) {
    Serial.println("\nMPU-6050 connection FAILED.");
    Serial.println("\nCheck your boards and connections. Reset the Arduino board to run the calibration sketch again.");
  }
  
  while (mpuConnection == 0) { }  //wait for MPU-6050 connection to be established. 

  Serial.println("\nMPU-6050 connection SUCCESSFUL!");
  delay(500);
  Serial.println("\n--------------------------------------------------------------");
  Serial.println("\nResetting MPU-6050 offsets.");
  
  // reset offsets
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);

  delay(500);
}

///////////////////////////////////   LOOP   ////////////////////////////////////
void loop() {
  if (state==0){
    Serial.println("\nReading accelerometer and gyroscope sensors for first time.");
    meansensors();
    state++;
    delay(1000);
  }

  if (state==1) {
    Serial.println("\nCalculating offsets");
    calibration();
    state++;
    delay(1000);
  }

  if (state==2) {
    meansensors();
    Serial.println("\nFINISHED!");
    Serial.println("\n==============================================================");
    Serial.println("\nRESULTS:");
    Serial.println("\nSensor data is listed in the format:\tacelX\tacelY\tacelZ\tgyroX\tgyroY\tgyroZ");
    Serial.print("\nSensor readings INCLUDING offsets:\t");
    Serial.print(mean_ax); 
    Serial.print("\t");
    Serial.print(mean_ay); 
    Serial.print("\t");
    Serial.print(mean_az); 
    Serial.print("\t");
    Serial.print(mean_gx); 
    Serial.print("\t");
    Serial.print(mean_gy); 
    Serial.print("\t");
    Serial.println(mean_gz);
    
    Serial.println("\nCompare with IDEAL sensor readings:\t0\t0\t16384\t0\t0\t0");
    Serial.println("\n--------------------------------------------------------------");
    
    Serial.println("\nYour MPU-6050 offsets:\n");
    Serial.print("mpu.setXAccelOffset(");
    Serial.print(ax_offset);
    Serial.println(");");
    Serial.print("mpu.setYAccelOffset(");
    Serial.print(ay_offset);
    Serial.println(");");
    Serial.print("mpu.setZAccelOffset(");
    Serial.print(az_offset);
    Serial.println(");");
    Serial.print("mpu.setXGyroOffset(");
    Serial.print(gx_offset);
    Serial.println(");");
    Serial.print("mpu.setYGyroOffset(");
    Serial.print(gy_offset);
    Serial.println(");");
    Serial.print("mpu.setZGyroOffset(");
    Serial.print(gz_offset);
    Serial.println(");");
    
    Serial.println("\nYou can copy and paste the above offsets directly into your sketch. :)");
    
    while (1);
  }
}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=gyro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(gyro_deadzone+1);

    if (abs(mean_gy)<=gyro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(gyro_deadzone+1);

    if (abs(mean_gz)<=gyro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(gyro_deadzone+1);

    if (ready==6) break;
  }
}