#include <Wire.h>

unsigned long ch1_count, ch2_count, ch3_count, ch4_count, current_count;
unsigned long esc1_count, esc2_count, esc3_count, esc4_count;
int ch[4] = {0, 1500, 0, 0};
int ch1_input, ch2_input, ch3_input, ch4_input;
bool ch1_state, ch2_state, ch3_state, ch4_state;
unsigned long loop_counter, esc_counter;
static int calibration[16] = {1068, 1496, 1888, 1204, 1480, 1788, 1180, 1500, 1812, 1220, 1500, 1800};
static bool invert_ch[4] = {false, false, false, true};
int start_motors = 0;

byte MPU_ADDR = 0x68;
int gyro_x_raw, gyro_y_raw, gyro_z_raw;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
int gyro_x, gyro_y, gyro_z;
long acc_x_raw, acc_y_raw, acc_z_raw, acc_sum;
float roll_acc, pitch_acc, yaw_acc;
int temp_raw;
volatile float pitch = 0, roll = 0, yaw = 0;
int cal;
volatile bool first_start = false;
int print_count;

//int pid_pitch_error, pid_roll_error, pid_yaw_error;
int pid_i_mem_pitch, pid_i_mem_roll, pid_i_mem_yaw;
int pid_d_prev_pitch_error, pid_d_prev_roll_error, pid_d_prev_yaw_error;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600);
  int LED_blink = 0;
  TWSR = (byte)11111000; //set TWI bit rate prescaler to 1
  TWBR = (byte)00001100; //set division factor for bit rate generator to 12 (400kHz I2C freq)
  DDRD = 0b11110000; //set pins 4-7 (ESCs) to output
  DDRB = 0b00010000; //set pin 12 (LED) to output
  PCICR = 0b00000001; //enable pin change interrupt on PCIE0
  PCMSK0 = 0b00001111; //enable interrupts on pins 8, 9, 10, 11

  while(convert_channels(3) == 1000 || convert_channels(4) == 1000){
    PORTD = 0b11110000; //set pins 4-7 high
    delayMicroseconds(1000); //send 1000us pulse to ESCs
    PORTD = 0b00000000; //set pins 4-7 low
    //give a 1/2 blink to indicate waiting for channels
    LED_blink ++;
    if (LED_blink < 500) PORTB = 0b00010000; //turn on LED
    else if (LED_blink >= 500 && LED_blink < 1000) PORTB = 0b00000000; //turn off LED
    else LED_blink = 0;
  }
  loop_counter = micros(); //update the loop counter

  MPU6050_setup();
  MPU6050_calibrate();
  delay(3);
  print_count = 0;
}


void loop() {
  // put your main code here, to run repeatedly:
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
  //get receiver data
  ch1_input = convert_channels(1);
  ch2_input = convert_channels(2);
  ch3_input = convert_channels(3);
  ch4_input = convert_channels(4);

  if (ch1_input == 1000 && ch4_input < 1050)start_motors = 1; //start the quadcopter motors
  if (start_motors = 1 && ch4_input > 1450){ //wait until the yaw stick is returned to center before starting motors
    start_motors = 2;
    //since the quad is not moving, reset the gyro roll and pitch angles
    roll = roll_acc;
    pitch = pitch_acc;
    //reset the D and I controller memory
    pid_d_prev_pitch_error = 0;
    pid_i_mem_pitch = 0;
    pid_d_prev_roll_error = 0;
    pid_i_mem_roll = 0;
    pid_d_prev_yaw_error = 0;
    pid_i_mem_yaw = 0;
  }
  if (start_motors = 2 && ch4_input > 1950) start_motors = 0;
  
  while (micros() - loop_counter < 4000); //wait until program loop counter reaches 4000 to achieve 250Hz program loop
  loop_counter = micros(); //update the loop counter
  
}

//ISR to read receiver channels
ISR(PCINT0_vect) {
  current_count = micros();
  //check channel 1
  if (ch1_state == false && PINB & 0b00000001) { //check if pin 8 is high (ch1)
    ch1_state = true;
    ch1_count = current_count;
  } else if (ch1_state == true && !(PINB & 0b00000001)) { //determine time ch1 was high
    ch1_state = false;
    ch[0] = current_count - ch1_count ;
  }
  //check channel 2
  if (ch2_state == false && PINB & 0b000000010) { //check if pin 9 is high (ch2)
    ch2_state = true;
    ch2_count = current_count;
  } else if (ch2_state == true && !(PINB & 0b00000010)) { //determine time ch2 was high
    ch2_state = false;
    ch[1] = current_count - ch2_count;
  }
  //check channel 3
  if (ch3_state == false && PINB & 0b000000100) { //check if pin 10 is high (ch3)
    ch3_state = true;
    ch3_count = current_count;
  } else if (ch3_state == true && !(PINB & 0b00000100)) { //determine time ch3 was high
    ch3_state = false;
    ch[2] = current_count - ch3_count;
  }
  //check channel 4
  if (ch4_state == false && PINB & 0b000001000) { //check if pin 11 is high (ch4)
    ch4_state = true;
    ch4_count = current_count;
  } else if (ch4_state == true && !(PINB & 0b00001000)) { //determine time ch4 was high
    ch4_state = false;
    ch[3] = current_count - ch4_count;
  }
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
    if (cal % 50 > 25) PORTB = 0b00010000;
    else PORTB = 0b00000000;
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

//convert receiver inputs to 1000-2000us pulses
int convert_channels(int channel) {
  int low, center, hi, raw;
  int offset;
  bool invert;
  raw = ch[channel - 1];
  low = calibration[(channel - 1) * 3];
  center = calibration[(channel - 1) * 3 + 1];
  hi = calibration[(channel - 1) * 3 + 2];
  invert = invert_ch[channel - 1];

  //interpolate between the low and center point
  if (raw < center) {
    if (raw < low) raw = low;
    offset = (float)(center - raw) * (float)500 / (center - low);
    if (invert == false) return 1500 - offset;
    else return 1500 + offset;
  }
  //interpolate between center and high point
  else if (raw > center) {
    if (raw > hi) raw = hi;
    offset = (float)(raw - center) * (float)500 / (hi - center);
    if (invert == false) return 1500 + offset;
    else return 1500 - offset;
  }
  else return 1500;
}

void ESC_send_pulses(){
  loop_counter = micros(); //base timer
  PORTD = 0b11110000; //set all esc ports to high
  esc1_count = loop_counter + ch1_input; //determine pulse length for esc1
  esc2_count = loop_counter + ch1_input; //determine pulse length for esc2
  esc3_count = loop_counter + ch1_input; //determine pulse length for esc3
  esc4_count = loop_counter + ch1_input; //determine pulse length for esc4
  while(PORTD > 0b00001111){ //wait until all esc ports are low
    esc_counter = micros();
    if (esc_counter >= esc1_count) PORTD &= 0b11101111; //send low pulse to front right esc
    if (esc_counter >= esc2_count) PORTD &= 0b11011111; //send low pulse to rear right esc
    if (esc_counter >= esc3_count) PORTD &= 0b10111111; //send low pulse to rear left esc
    if (esc_counter >= esc4_count) PORTD &= 0b01111111; //send low pulse to front left esc
  }
}
