#include<Wire.h>

double accelX,accelY,accelZ,temperature,gyroX,gyroY,gyroZ,gyro_x_cal,gyro_y_cal,gyro_z_cal; //These will be the raw data from the MPU6050.
uint32_t timer; //it's a timer, saved as a big-ass unsigned int.  We use it to save times from the "micros()" command and subtract the present time in microseconds from the time stored in timer to calculate the time for each loop.
double roll, pitch ,yaw; //These are the angles in the complementary filter
float rollangle,pitchangle,acc_total_vector;
int cal_int;
void setup() {
  // Set up MPU 6050:
  Serial.begin(115200);
  Wire.begin();

  setupMPU();   
  Serial.println("Calibrating Gyroscope......");
  for(cal_int=1;cal_int<=2000;cal_int++)
  {
   recordRegisters();
   gyro_x_cal += gyroX;
   gyro_y_cal  += gyroY ;
   gyro_z_cal += gyroZ;
  }
  Serial.println("Calibration Done..!!!");
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
 
  //start a timer
  timer = micros();
}

void loop() {
//Now begins the main loop.
  //Collect raw data from the sensor.
  recordRegisters();
  gyroX = gyroX / 65.5;
  gyroY = gyroY / 65.5;
  gyroZ = gyroZ / 65.5;

  accelX = accelX / 4096.0;
  accelY = accelY / 4096.0;
  accelZ= accelZ / 4096.0;
 
  double dt = (double)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
  timer = micros(); //start the timer again so that we can calculate the next dt.

  //the next two lines calculate the orientation of the accelerometer relative to the earth and convert the output of atan2 from radians to degrees
  //We will use this data to correct any cumulative errors in the orientation that the gyroscope develops.
  rollangle=atan2(accelY,sqrt((accelZ*accelZ)+(accelX*accelX)))*180/PI; // FORMULA FOUND ON INTERNET
  pitchangle=atan2(-accelZ,accelX)*180/PI; //FORMULA FOUND ON INTERNET

  //acc_total_vector = sqrt((accelX*accelX)+(accelY*accelY)+(accelZ*accelZ));  //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  //pitchangle = asin((float)accelY/acc_total_vector)* 57.296;       //Calculate the pitch angle
  //rollangle = asin((float)accelX/acc_total_vector)* -57.296; 
 

  //THE COMPLEMENTARY FILTER
  //This filter calculates the angle based MOSTLY on integrating the angular velocity to an angular displacement.
  //dt, recall, is the time between gathering data from the MPU6050.  We'll pretend that the
  //angular velocity has remained constant over the time dt, and multiply angular velocity by
  //time to get displacement.
  //The filter then adds a small correcting factor from the accelerometer ("roll" or "pitch"), so the gyroscope knows which way is down.
  roll = 0.99*(roll+ gyroX * dt) + 0.01 * rollangle; // Calculate the angle using a Complimentary filter
  pitch = 0.99*(pitch + gyroY * dt) + 0.01 * pitchangle;
  yaw=0.99*(yaw+gyroZ*dt);
 
 
  Serial.print("roll  ");
  Serial.print(roll);
  Serial.print("   pitch  ");
  Serial.print(pitch);
  Serial.print("   yaw    ");
  Serial.println(yaw);

}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4)
  Wire.write(0x08); //Setting the gyro to full scale +/- 500deg./s
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5)
  Wire.write(0x10); //Setting the accel to +/- 8g
  Wire.endTransmission();
 
  Wire.beginTransmission(0b1101000);                                      //Start communication with the address found during search
  Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                    //End the transmission with the gyro   
}
void recordRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,14); //Request Accel Registers (3B - 40)
  while(Wire.available() < 14);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
  temperature=Wire.read()<<8|Wire.read();
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ

  if(cal_int == 2000)
  {
    gyroX -= gyro_x_cal;
    gyroY -= gyro_y_cal;
    gyroZ -= gyro_z_cal;
   
  }
}
