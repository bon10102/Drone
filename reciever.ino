unsigned long ch1_count, ch2_count, ch3_count, ch4_count, current_count;
int ch[4] = {0, 1500, 0, 0};
bool ch1_state, ch2_state, ch3_state, ch4_state;
long loop_counter;
static int calibration[16] = {1068, 1496, 1888, 1204, 1480, 1788, 1180, 1500, 1812, 1220, 1500, 1800};
static bool invert_ch[4] = {false, false, false, false};
bool start;

void setup() {
  Serial.begin(9600);
  pinMode(12, OUTPUT);
  PCICR = 0b00000001; //enable pin change interrupt on PCIE0
  PCMSK0 = 0b00001111; //enable interrupts on pins 8, 9, 10, 11
}

void loop() {
  //print the channel data
  if (convert_channels(1) == 1000 && convert_channels(4) > 1896)start = true;
  else if (convert_channels(1) == 1000 && convert_channels(4) < 1016)start = false;
  Serial.print(F("Start "));
  Serial.print(start);
  Serial.print(" || ");
  Serial.print(F("Throttle "));
  Serial.print(convert_channels(1));
  Serial.print(" || ");
  Serial.print(F("Roll "));
  Serial.print(convert_channels(2));
  Serial.print(" || ");
  Serial.print(F("Pitch "));
  Serial.print(convert_channels(3));
  Serial.print(" || ");
  Serial.print(F("Yaw "));
  Serial.print(convert_channels(4));
  Serial.println();
  delay(500);
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
