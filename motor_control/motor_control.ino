#define ENCODER 7

void setup() {
  // put your setup code here, to run once:
 Serial.begin(9600);
 pinMode(ENCODER, INPUT);
 
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long duration = pulseIn(ENCODER, HIGH);
  Serial.println(duration);
}
