#include <Wire.h>

#include <Adafruit_I2CDevice.h>

#include <Adafruit_I2CRegister.h>

#include "Adafruit_MCP9600.h"

 

#define I2C_ADDRESS (0x67)

 

Adafruit_MCP9600 mcp;      

 

float averageReading = 0;

const int bufferSize = 10;  // Size of the rolling buffer

float readings[bufferSize];

int errorCorrectioncurrentIndex = 0;

int threshold = 5;  // Adjust this threshold as needed

float average = 0;  // Variable to store the average

int validReadingCount = 0;  // Count of valid readings within the buffer

 

//Inputs and outputs

gpio_num_t ELEMENT_firing_pin = GPIO_NUM_19;

gpio_num_t ELEMENT_firing_pin_2 = GPIO_NUM_4;

gpio_num_t ELEMENT_firing_pin_3 = GPIO_NUM_18;

 

gpio_num_t zero_cross = GPIO_NUM_26;

gpio_num_t zero_cross_2 = GPIO_NUM_27;

gpio_num_t zero_cross_3 = GPIO_NUM_5;

 

unsigned long currentmicros2;

unsigned long previousmicros = 0;

unsigned long currentmicros = 0;

int temp_read_Delay = 1000;

 

unsigned long Next_Firing_Pulse_P1_ON = 0;

unsigned long Next_Firing_Pulse_P2_ON = 0;

unsigned long Next_Firing_Pulse_P3_ON = 0;

 

unsigned long Next_Firing_Pulse_P1_OFF = 0;

unsigned long Next_Firing_Pulse_P2_OFF = 0;

unsigned long Next_Firing_Pulse_P3_OFF = 0;

int maximum_firing_delay = 8500;

int firing_delay = 8000;

float kp = 820;

float p_value = 0;

float error;

double setRateofChange = 10.00;

unsigned long previousmicros_TempUpdate = 0;

unsigned long currentmicros_forTempUpdate = 0;

int temp_update_Delay = 250000;

 

//float temperatureReadings[60]; // Array to store 60 temperature readings

int readingIndex = 0;

int numReadings = 0;

unsigned long lastTenSecondCheck = 0; // Time of the last 10-second check

// An array of 60 elements to store the temperature readings.

float temperatureReadings[240];

 

// A variable to store the current index of the array.

int currentIndex = 0;

 

//Zero Crossing Interrupt Function

void IRAM_ATTR zero_crossing() {

  delayMicroseconds(20);

  if (gpio_get_level(zero_cross) == LOW)

    Next_Firing_Pulse_P1_ON = micros() + firing_delay;

    Next_Firing_Pulse_P1_OFF = Next_Firing_Pulse_P1_ON + 100;

}

 

void IRAM_ATTR zero_crossing_2() {

  delayMicroseconds(20);

  if (gpio_get_level(zero_cross_2) == LOW)

    Next_Firing_Pulse_P2_ON = micros() + firing_delay;

    Next_Firing_Pulse_P2_OFF = Next_Firing_Pulse_P2_ON + 100;

}

 

void IRAM_ATTR zero_crossing_3() {

  delayMicroseconds(20);

  if (gpio_get_level(zero_cross_3) == LOW)

    Next_Firing_Pulse_P3_ON = micros() + firing_delay;

    Next_Firing_Pulse_P3_OFF = Next_Firing_Pulse_P3_ON + 100;

}

 

void setup() {

  Serial.begin(115200);

  pinMode (zero_cross, INPUT);

  pinMode (ELEMENT_firing_pin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(zero_cross), zero_crossing, FALLING);

  pinMode (zero_cross_2, INPUT);

  pinMode (ELEMENT_firing_pin_2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(zero_cross_2), zero_crossing_2, FALLING);

 

  pinMode (zero_cross_3, INPUT);

  pinMode (ELEMENT_firing_pin_3, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(zero_cross_3), zero_crossing_3, FALLING);

 

 

  //MCP9600

  while (!Serial) {

    delay(10);

  }

  Serial.println("MCP9600 ");

 

  /* Initialise the driver with I2C_ADDRESS and the default I2C bus. */

  if (! mcp.begin(I2C_ADDRESS)) {

    Serial.println("Sensor not found. Check wiring!");

    while (1);

  }

 

  Serial.println("Found MCP9600!");

 

  mcp.setADCresolution(MCP9600_ADCRESOLUTION_18);

// Serial.print("ADC resolution set to ");

    switch (mcp.getADCresolution()) {

    case MCP9600_ADCRESOLUTION_18:   Serial.print("18"); break;

    case MCP9600_ADCRESOLUTION_16:   Serial.print("16"); break;

    case MCP9600_ADCRESOLUTION_14:   Serial.print("14"); break;

    case MCP9600_ADCRESOLUTION_12:   Serial.print("12"); break;

  }

// Serial.println(" bits");

 

  mcp.setThermocoupleType(MCP9600_TYPE_K);

    Serial.print("Thermocouple type set to ");

    switch (mcp.getThermocoupleType()) {

    case MCP9600_TYPE_K:  Serial.print("K"); break;

    case MCP9600_TYPE_J:  Serial.print("J"); break;

  }

  Serial.println(" type");

 

  mcp.setFilterCoefficient(3);

  mcp.setAlertTemperature(1, 30);

  //Serial.print("Alert #1 temperature set to ");

// Serial.println(mcp.getAlertTemperature(1));

  mcp.configureAlert(1, true, true);  // alert 1 enabled, rising temp

 

  mcp.enable(true);

 

  Serial.println(F("-----"));

}

 

void loop() {

 

 

 

 

  currentmicros_forTempUpdate = micros();

 

  // Record a temperature reading every second

  if (currentmicros_forTempUpdate - previousmicros_TempUpdate >= temp_update_Delay) {

    previousmicros_TempUpdate += temp_update_Delay;

 

    float reading = mcp.readThermocouple();

 

  // Check if the new reading deviates from the previous reading by more than the threshold

  if (!isErroneous(reading, errorCorrectioncurrentIndex)) {

    // Store the reading in the rolling buffer

    readings[errorCorrectioncurrentIndex] = reading;

 

    // Move to the next position in the rolling bufferlll

    errorCorrectioncurrentIndex = (errorCorrectioncurrentIndex + 1) % bufferSize;

 

    if (validReadingCount < bufferSize) {

      validReadingCount++;

    }

 

    // Calculate the updated average

    averageReading = calculateAverage();

    //averageReading =average;

  }

 

    // Read the temperature sensor and store the reading in the array at the current index.

    //float temperatureReading =  mcp.readThermocouple();

    temperatureReadings[currentIndex] = averageReading;//temperatureReading;

   

    // Increment the current index.

    currentIndex++;

  

 

    // If the current index is equal to 60, delete the first element in the array and shift all the other elements to the left.

    if (currentIndex == 240) {

      for (int i = 0; i < 239; i++) {

        temperatureReadings[i] = temperatureReadings[i + 1];

      }

 

      currentIndex = 239;

    }

 

    // Check if 10 seconds have passed

    if (micros() >= 70000000) {

      if (currentmicros_forTempUpdate - lastTenSecondCheck >= 2000000) {

        float CurrentRateofChange = temperatureReadings[59] - temperatureReadings[1];

        //Serial.println(CurrentRateofChange);

        error = setRateofChange - CurrentRateofChange;

        p_value = kp * error;

        if (p_value < 0)

          p_value = 0;

        if (p_value > maximum_firing_delay)

          p_value = maximum_firing_delay;

         firing_delay = (maximum_firing_delay - p_value);

        if (firing_delay <= 6500) {

          firing_delay = 7000;}

       //firing_delay = (maximum_firing_delay - p_value);

        Serial.print(",CurrentRateofChange =, " + String(CurrentRateofChange));

        Serial.print(",Error =, " + String(error));

        Serial.print(",Temp =, " + String(averageReading));

        Serial.print(",Temp =, " + String(reading));

        Serial.print(",p_value =, " + String(p_value));

        Serial.println(",firingdelay =, " + String((firing_delay)));

 

        // Update the last 10-second check time

      

        

        

       // }

        lastTenSecondCheck = currentmicros_forTempUpdate;

       //firing_delay = 8500; //hard coded firing delay

      }

    }

  }

 

  if (Next_Firing_Pulse_P1_ON < micros() ) {

    digitalWrite(ELEMENT_firing_pin, HIGH);

    Next_Firing_Pulse_P1_ON = (micros()+ 10000);

  }

 

  if(Next_Firing_Pulse_P1_OFF < micros() ){

    digitalWrite(ELEMENT_firing_pin, LOW);

    Next_Firing_Pulse_P1_OFF = (micros()+ 10000);

  }

 

 

if (Next_Firing_Pulse_P2_ON < micros() ) {

    digitalWrite(ELEMENT_firing_pin_2, HIGH);

    Next_Firing_Pulse_P2_ON = (micros()+ 10000);

  }

 

  if(Next_Firing_Pulse_P2_OFF < micros() ){

    digitalWrite(ELEMENT_firing_pin_2, LOW);

    Next_Firing_Pulse_P2_OFF = (micros()+ 10000);

  }

 

 

  if (Next_Firing_Pulse_P3_ON < micros() ) {

    digitalWrite(ELEMENT_firing_pin_3, HIGH);

    Next_Firing_Pulse_P3_ON = (micros()+ 10000);

  }

 

  if(Next_Firing_Pulse_P3_OFF < micros() ){

    digitalWrite(ELEMENT_firing_pin_3, LOW);

    Next_Firing_Pulse_P3_OFF = (micros()+ 10000);

  }

 

 

}

bool isErroneous(float newValue, int index) {

  if (index == 0) {

    return false;  // No previous reading to compare with.

  }

 

  // Get the previous reading from the rolling buffer

  float previousValue = readings[(index - 1 + bufferSize) % bufferSize];

 

  // Check if the new reading deviates from the previous reading by more than the threshold

  if (abs(newValue - previousValue) > threshold) {

    return true;  // It's an erroneous reading.

  }

  else {

  return false;}  // It's a valid reading.

}

 

 

 

float calculateAverage() {

  float sum = 0;

  for (int i = 0; i < validReadingCount; i++) {

    sum += readings[i];

  }

 

  return sum / validReadingCount;

 

  //Serial.print("Average: ");

  //Serial.println(average);

}

 

/*float calculateAverage() {

  float sum = 0;

  for (int i = 0; i < validReadingCount; i++) {

    if (!isErroneous(readings[i], errorCorrectioncurrentIndex)) {

      sum += readings[i];

    }

  }

 

  return sum / validReadingCount;

}*/


 
