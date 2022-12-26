#include <Wire.h>
#define RPM 170
//#define ppr 290 //42 - rpm
#define ppr 375 //105 - rpm
#define Kp  0.461121665031208
#define Ki  3.07568150575186
#define Ts  10/1000 //10ms



int pin = 7;
unsigned long duration;

int counter;
unsigned long int timer;

// motor variables
volatile float period = 0;
volatile float fs = 0;
volatile float fm = 0;
volatile float rpm = 0;
volatile float ppr_avg = 0.0;
//volatile float fm = RPM/60;
//volatile float ppr = fs/fm;

// control-pi variables
volatile float sp = 30;	// set point rpm
volatile float Err_1 = 0;
volatile float Err = 0;
volatile float PI_ = 0;


// kalman-filter variables
volatile float x_1 = 0;
volatile float x_0 = 0;  // position at initial time
volatile float p_1 = 0;  
volatile float p_0 = 10; // variance at initial position
volatile float k = 0;  // kalman gain
volatile float z = 0;  // rpm measurments
volatile float r = 20; // variance of the "sensor" to measure rpm
volatile float q = 1;
volatile float duty_cycle = 0;
volatile float data = sp;

ISR(TIMER1_COMPA_vect)
{
  
  // rpm measurment from encoder square signal
  period = 2*duration;

  if (duration>11305 or duration==0){
    fs = 0;
  }
  fs = 1000000/period;
  fm = fs/ppr;
  rpm = fm*60;

  if (rpm > 190){
    rpm = 0;
  }

  
  // kalman filter to improve the rpm estimation
  x_1 = x_0;
  p_1 = p_0 + q;
  
  z = rpm;
  k = p_1/(p_1 + r);

  x_0 = x_1 + k*(z - x_1);
  p_0 = (1 - k)*p_1;

  
  // PI control to change the rpm
  Err = sp - x_0; 
  PI_ = Kp*Err - (Kp - Ki*Ts)*Err_1 + PI_;
  Err_1 = Err;

  if (PI_ < 0){
    PI_ = 0.0;
  }else if (PI_>12){
    PI_ = 12.0;
  }

  duty_cycle = (PI_/12.0)*255.0;

  analogWrite(3,duty_cycle);

  Serial.print(z);
  Serial.print(" ");
  Serial.println(x_0); 
}

void setup()
{

  analogWrite(3,255);
  
  Serial.begin(115200);
  pinMode(pin, INPUT);
  noInterrupts();
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // set compare match register for 100 Hz increments
  OCR1A = 19999; // = 16000000 / (8 * 100) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 8 prescaler
  TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  interrupts();             // enable all interrupts


}

void loop()
{
  //timer = millis();

  if(Serial.available() != 0 ){
    data = Serial.parseFloat(SKIP_WHITESPACE);
    sp = data;
  }

  //while(!Serial.available()){
  duration = pulseIn(pin, HIGH);
  //}
  
  //sp = Serial.parseFloat(SKIP_WHITESPACE);
  //Serial.print(sp + 1);
  
}
