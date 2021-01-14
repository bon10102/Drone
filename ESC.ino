unsigned long ch1_count, ch2_count, ch3_count, ch4_count, current_count;
unsigned long esc1_count, esc2_count, esc3_count, esc4_count;
int ch[4] = {0, 1500, 0, 0};
int ch1_input, ch2_input, ch3_input, ch4_input;
bool ch1_state, ch2_state, ch3_state, ch4_state;
unsigned long loop_counter, esc_counter;
static int calibration[16] = {1068, 1496, 1888, 1204, 1480, 1788, 1180, 1500, 1812, 1220, 1500, 1800};
static bool invert_ch[4] = {false, false, false, false};
bool start;

void setup() {
  int start = 0;
  Serial.begin(9600);
  pinMode(12, OUTPUT);
  DDRD = 0b11110000; //set pins 4-7 (ESCs) to output
  DDRB = 0b00010000; //set pin 12 (LED) to output
  PCICR = 0b00000001; //enable pin change interrupt on PCIE0
  PCMSK0 = 0b00001111; //enable interrupts on pins 8, 9, 10, 11

  PORTB = 0b00010000; //turn the LED on to indicate waiting for reciever
  while(convert_channels(3) == 1000 || convert_channels(4) == 1000){
    Serial.print(convert_channels(3));
    PORTD = 0b11110000; //set pins 4-7 high
    delayMicroseconds(1000); //send 1000us pulse to ESCs
    PORTD = 0b00000000; //set pins 4-7 low
  }
  PORTB = 0b00000000; //turn the LED off
  loop_counter = micros();
}

void loop() {
  //print the channel data
  if (convert_channels(1) == 1000 && convert_channels(4) > 1896)start = true;
  else if (convert_channels(1) == 1000 && convert_channels(4) < 1020)start = false;
  ch1_input = convert_channels(1);
  ch2_input = convert_channels(2);
  ch3_input = convert_channels(3);
  ch4_input = convert_channels(4);
  while (loop_counter + 4000 > micros());
  loop_counter = micros();
  ESC_send_pulses();
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
