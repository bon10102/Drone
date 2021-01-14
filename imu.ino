#include <Wire.h>

byte MPU_ADDR = 0x68;
int gyro_x_raw, gyro_y_raw, gyro_z_raw;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
int gyro_x, gyro_y, gyro_z;
long acc_x_raw, acc_y_raw, acc_z_raw, acc_sum;
float roll_acc, pitch_acc, yaw_acc;
int temp_raw;
volatile float pitch = 0, roll = 0, yaw = 0;
int cal;
long loop_counter;
volatile bool first_start = false;
int print_count;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  TWSR = (byte)11111000; //set TWI bit rate prescaler to 1
  TWBR = (byte)00001100; //set division factor for bit rate generator to 12 (400kHz I2C freq)
  MPU6050_setup();
  Serial.println(F("calibrating, do not move gyro"));
  MPU6050_calibrate();
  Serial.println(F("calibration complete!"));
  delay(3);
  print_count = 0;
  loop_counter = micros(); //update the loop counter
}

void loop() {
  MPU6050_getData();
  //integrate gyro values to get angles
  //0.0006107 = 1/250 Hz/65.5 LSB/deg/s
  roll += gyro_x * 0.00006107;
  pitch += gyro_y * 0.00006107;
  yaw += gyro_z * 0.00006107;

  //transfer pitch and roll axis if quadcopter is yawed
  //0.000001065 = 0.0006107*(pi/180deg)
  pitch += roll * sin(gyro_z * 0.000001065);
  roll -= pitch * sin(gyro_z * 0.000001065);

  //calculate angle based on accelerometer reading for roll and pitch (doesn't work for yaw)
  //57.296 = 180deg/pi
  acc_sum = sqrt((acc_x_raw * acc_x_raw) + (acc_y_raw * acc_y_raw) + (acc_z_raw * acc_z_raw));
  roll_acc = asin((float)acc_y_raw / acc_sum) * 57.296;
  pitch_acc = asin((float)acc_x_raw / acc_sum) * 57.296;

  roll_acc += 11;
  pitch_acc -= 17;

  //apply complementary filter to reduce gyro drift
  if (first_start) {
    roll = roll * 0.9996 + roll_acc * 0.0004;
    pitch = pitch * 0.9996 + pitch_acc * 0.0004;
  } else {
    roll = roll_acc;
    pitch = pitch_acc;
    first_start = true;
  }

  //print the quadcopter orientation
  if (print_count == 0) Serial.print(F("Pitch: "));
  if (print_count == 1) Serial.print((int)pitch);
  if (print_count == 2) Serial.print(F(" || "));
  if (print_count == 3) Serial.print(F("Roll: "));
  if (print_count == 4) Serial.print((int)roll);
  if (print_count == 5) Serial.print(F(" || "));
  if (print_count == 6) Serial.print(F("Yaw: "));
  if (print_count == 7) Serial.print((int)yaw);
  if (print_count == 8) Serial.println();

  print_count ++;
  if (print_count > 40) print_count = 0;
  
  while (micros() - loop_counter < 4000); //wait until program loop counter reaches 4000 to achieve 250Hz program loop
  loop_counter = micros(); //update the loop counter
}

void MPU6050_setup() {
  //variables to store gyro registers
  byte PWR_MGMT_1 = 0x6B;
  byte GYRO_CONFIG = 0x1B;
  byte ACCEL_CONFIG = 0x1C;
  byte CONFIG = 0x1A;
  //configure gyro clock (internal 8MHz)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  //configure gyro range (+/- 500 deg/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(GYRO_CONFIG);
  Wire.write(0x08);
  Wire.endTransmission();
  //configure accelerometer range (+/- 8g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_CONFIG);
  Wire.write(0x10);
  Wire.endTransmission();
  //apply 44Hz low pass filter (min RPM of motor is 1293)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(CONFIG);
  Wire.write(0x03);
  Wire.endTransmission();
}

void MPU6050_calibrate() {
  //initalize calibration variables
  for (cal = 0; cal < 2000; cal++) {
    MPU6050_getData();
    gyro_x_cal += gyro_x_raw;
    gyro_y_cal += gyro_y_raw;
    gyro_z_cal += gyro_z_raw;
    delay(3);
  }
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
}

void MPU6050_getData() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); //request data from first register storing output
  Wire.endTransmission();
  Wire.requestFrom((int)MPU_ADDR, 14); //request acceleration, temp, and gyro data (14 bytes total)
  while (Wire.available() < 14); //store data in buffer
  acc_x_raw = Wire.read() << 8 | Wire.read(); //get x acceleration and concatenate high and low order bytes
  acc_y_raw = Wire.read() << 8 | Wire.read(); //get y acceleration and concatenate high and low order bytes
  acc_z_raw = Wire.read() << 8 | Wire.read(); //get z acceleration and concatenate high and low order bytes
  temp_raw = Wire.read() << 8 | Wire.read(); //get temperature and concatenate high and low order bytes
  gyro_x_raw = Wire.read() << 8 | Wire.read(); //get x angular velocity and concatenate high and low order bytes
  gyro_y_raw = Wire.read() << 8 | Wire.read(); //get y angular velocity and concatenate high and low order bytes
  gyro_z_raw = Wire.read() << 8 | Wire.read(); //get z angular velocy and concatenate high and low order bytes

  //apply calibration values to obtain gyro values
  if (cal >= 2000) {
    gyro_x = gyro_x_raw - gyro_x_cal;
    gyro_y = (gyro_y_raw - gyro_y_cal) * -1;
    gyro_z = gyro_z_raw - gyro_z_cal;
  }
}
