int Valve1  = 3;
int Valve2 = 4;
int Valve3 = 5;
float tank_pressureSensor;
float tank_pressureSensor2;
int currentState = 1;
unsigned long startTime;
unsigned long currentMillis;
const long heating_Time = 5400000;
int reset_Zero = 0;
const long run_Time = 19800000;
const long waiting_Time = 900000;
const long cooling_Time = 300000;
unsigned long finishing_purging_time;
const long refilling = 30000;

void setup() {
  Serial.begin(9600);
  pinMode(Valve1, OUTPUT);
  pinMode(Valve2, OUTPUT);
  pinMode(Valve3, OUTPUT);
  startTime = millis();
}

void loop() {
  // make 16 readings and average them (reduces some noise) you can tune the number 16 of course
  int count = 16;
  int raw = 0;
  for (int i = 0; i < count; i++) raw += analogRead(A5); // return 0..1023 representing 0..5V
  raw = raw / count;
  int count2 = 16;
  int raw2 = 0;
  for (int i = 0; i < count2; i++) raw2 += analogRead(A2); // return 0..1023 representing 0..5V
  raw2 = raw2 / count2;

  // CONVERT TO VOLTAGE
  float voltage = 5.0 * raw / 1023; // voltage = 0..5V;  we do the math in millivolts!!
  float voltage2 = 5.0 * raw2 / 1023; // voltage2 = 0..5V;  we do the math in millivolts!!
  // INTERPRET VOLTAGES
  if (voltage < 0.5)
  {
    Serial.println("open circuit");
  }
  else if (voltage < 4.5)  // between 0.5 and 4.5 now...
  {
    float after_chiller_pressureSensor = mapFloat(voltage, 0.5, 4.5, -29.52, 61.08);    // variation on the Arduino map() function
    //Serial.print(voltage);
    Serial.print("after_chiller_pressureSensor =  ");
    Serial.print(after_chiller_pressureSensor - 2.8, 2); // 3 decimal
    tank_pressureSensor2 = mapFloat(voltage2, 0.5, 4.5, -29.52, 61.08);    // variation on the Arduino map() function
    //Serial.print(voltage);
    Serial.print(", ");
    Serial.print("tank_pressureSensor =  ");
    tank_pressureSensor = tank_pressureSensor2 -2.8;
    Serial.println(tank_pressureSensor, 2); // 3 decimals
  }
  else {
    Serial.println("Signal too high!!");
  }

  if (currentState == 1)
    S1_heating();
  else if (currentState == 2)
    S2_filling();
  else if (currentState == 3)
    S3_purging();
  else if (currentState == 4)
    S4_waiting();
  else if (currentState == 5)
    S5_cooling();
  else if (currentState == 6)
    S6_lastRefill();
  else if (currentState == 7)
    S7_finishing();
}

void S1_heating() {
  currentMillis = millis();
  Serial.print("currentMillis= ");
  Serial.print(currentMillis);
  Serial.print(", heating state  ");
  digitalWrite (Valve1, HIGH);
  digitalWrite (Valve2, HIGH);
  digitalWrite (Valve3, LOW);
  unsigned long difference1;
  difference1 = currentMillis - startTime;
  if (difference1 >= heating_Time) {
    currentState = 2;
  }
}
void S2_filling() {
  currentMillis = millis();
  Serial.print("currentMillis= ");
  Serial.print(currentMillis);
  Serial.print(", filling state");
  digitalWrite (Valve1, LOW);
  digitalWrite (Valve2, LOW);
  digitalWrite (Valve3, HIGH);
  if (tank_pressureSensor >= -10.00) {
    currentState = 3;
  }
}

void S3_purging() {
  currentMillis = millis();
  Serial.print ("currentMillis =  ");
  Serial.print (currentMillis);
  Serial.print(", purging state");
  digitalWrite (Valve1, HIGH);
  digitalWrite (Valve2, HIGH);
  digitalWrite (Valve3, LOW);
  if (tank_pressureSensor <= -26.00) {
    currentState = 4;
    startTime = currentMillis;
  }
}

void S4_waiting() {
  currentMillis = millis();
  Serial.print ("currentMillis =  ");
  Serial.print (currentMillis);
  Serial.print(", waiting  ");
  digitalWrite (Valve1, HIGH);
  digitalWrite (Valve2, HIGH);
  digitalWrite (Valve3, LOW);
  //Serial.print ("startTime =  ");
  //Serial.print (startTime);
  unsigned long difference;
  difference = currentMillis - startTime;
  //Serial.print ("difference=  ");
  //Serial.print (difference);
  if (currentMillis >= run_Time) {
    currentState = 5;
    finishing_purging_time = currentMillis;
  }
  else if (difference >= waiting_Time) {
    currentState = 2;
    difference = reset_Zero;
  }
}

void S5_cooling(){
  currentMillis = millis();
  Serial.print ("currentMillis =  ");
  Serial.print (currentMillis);
  Serial.print (", cooling");
  digitalWrite (Valve1, HIGH);
  digitalWrite (Valve2, HIGH);
  digitalWrite (Valve3, LOW);
  const long switch_to_lastRefill = cooling_Time + finishing_purging_time;
  if (currentMillis >= switch_to_lastRefill) {
    currentState = 6;
  }
  }
void S6_lastRefill(){
  currentMillis = millis();
  Serial.print ("currentMillis =  ");
  Serial.print (currentMillis);
  Serial.print (", last refilling");
  digitalWrite (Valve1, LOW);
  digitalWrite (Valve2, LOW);
  digitalWrite (Valve3, HIGH);
  const long switch_to_finish = cooling_Time + finishing_purging_time + refilling;
  if (currentMillis >= switch_to_finish) {
    currentState = 7;
 
}}

void S7_finishing() {
  currentMillis = millis();
  Serial.print ("currentMillis =  ");
  Serial.print (currentMillis);
  digitalWrite (Valve1, LOW);
  digitalWrite (Valve2, LOW);
  digitalWrite (Valve3, LOW);
  Serial.print(", Rejuvination process completed");
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
