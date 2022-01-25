#include "max6675.h"
int thermoDO = 19;
int thermoCS = 23;
int thermoCLK = 4;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

//Inputs and outputs
gpio_num_t ELEMENT_firing_pin = GPIO_NUM_21; // THIS IS PIN IS CONNECTED TO THE GATE OF THE TRIAC 
gpio_num_t zero_cross = GPIO_NUM_22; // THIS IS FOR THE ZERO CROSSING DETECTION 

unsigned long previousMillis = 0; 
unsigned long currentMillis = 0;
int temp_read_Delay = 500;

int last_CH1_state = 0;
bool zero_cross_detected = false;
const int maximum_firing_delay = 9000;
int setpoint = 250;
double temperature = 0.00;
int kp = 850;
float p_value = 0;
float error;

//Zero Crossing Interrupt Function
void IRAM_ATTR zero_crossing()
{
  
 delayMicroseconds(10);
 if (gpio_get_level(zero_cross))
    zero_cross_detected = true; 
    
    }
  

void setup() 
{
  
  Serial.begin(9600);
  pinMode (zero_cross, INPUT); 
  pinMode (ELEMENT_firing_pin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(zero_cross), zero_crossing, RISING); 
    
    }
 



void loop() 
{
  
   currentMillis = millis();
   if(currentMillis - previousMillis >= temp_read_Delay)
   {
      previousMillis += temp_read_Delay;
      temperature = thermocouple.readCelsius();
  

      error = setpoint-temperature;
      p_value = kp*error;
  
      if(p_value < 0)      
          p_value = 0;       
      if(p_value > maximum_firing_delay)      
          p_value = maximum_firing_delay; 
  
      Serial.print(",SP =, " + String(setpoint));
      Serial.print(",Temp =, " + String(temperature));
      Serial.print(",p_value =, " + String(p_value));
      Serial.println(",firingdelay =, " + String((maximum_firing_delay - p_value)/100));
      
      }
  
    if (zero_cross_detected )
    {
      zero_cross_detected = false;
      delayMicroseconds(maximum_firing_delay - p_value);
      digitalWrite(ELEMENT_firing_pin,HIGH);
      delayMicroseconds(100);
      digitalWrite(ELEMENT_firing_pin,LOW);
    
      }

    }
