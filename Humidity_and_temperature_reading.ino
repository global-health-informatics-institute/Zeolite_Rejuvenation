
#include <Wire.h>

#define si7021 0x40 //set humidity sensor address

double Dryer_Humidity = 0.00;
double Drum_Humidity = 0.00;
double Dryer_Temperature = 0.00;
double Drum_Temperature = 0.00;
double Dew_point = 0.00;
long lastMsg = 0;
int X0,X1,temp;
double X,X_out;

const int MeasureHumid = 0xE5; //Command to sent to the si7021 to read the humidity 

const int segment1_1 = 5;
const int segment1_2= 18; 
const int segment1_3 =19;
const int segment1_4 =2;
const int segment1_5 =4;
const int segment1_6 =17;
const int segment1_7 =16;
const int segment2_1 =32;
const int segment2_2 =33;
const int segment2_3 =25;
const int segment2_4 =26;
const int segment2_5 =27;
const int segment2_6 =12;
const int segment2_7 =14;
const int ground  = 15;
int seg1;
int seg2;

bool WireEnd() 
{
  unsigned char err;
  err = Wire.endTransmission();
  if( err ) 
  {
    Serial.print("Error: ");
    Serial.println(err);
      
      }
  return err;
  
    }

double GetHumid(int SDA_Pin, int SLC_pin) 
{
  unsigned int data[2];
  Wire.begin(SDA_Pin,SLC_pin,10000);
  Wire.beginTransmission(si7021);
  Wire.write(0xF5);
  WireEnd();
  delay(20);
  Wire.requestFrom(si7021, 2);
  
  if(Wire.available() == 2)
  {
    data[0] = Wire.read();
    data[1] = Wire.read();
      
      }
      
  unsigned int temp = ((data[0] << 8) + data[1]);
  float humidity = ((125.0 * temp) / 65536.0) - 6;
  return humidity;
}


double GetTemp(int SDA_Pin, int SLC_pin)
{
  Wire.begin(SDA_Pin,SLC_pin,10000);
  Wire.beginTransmission(si7021);
  Wire.write(0xE3);
  Wire.endTransmission();
  Wire.requestFrom(si7021,2);
  
  if(Wire.available()<=2);
  {
     X0 = Wire.read();
     X1 = Wire.read();
     X0 = X0<<8;
     X_out = X0+X1;
  
      }
      
  /**Calculate temperature**/
  X=(175.72*X_out)/65536;
  X=X-46.85;
  return X;
    
    }

void setup() 
{
  Serial.begin(115200);

  pinMode(segment1_1, OUTPUT);
  pinMode(segment1_2, OUTPUT);
  pinMode(segment1_3, OUTPUT);
  pinMode(segment1_4, OUTPUT);
  pinMode(segment1_5, OUTPUT);
  pinMode(segment1_6, OUTPUT);
  pinMode(segment1_7, OUTPUT);
  pinMode(segment2_1, OUTPUT);
  pinMode(segment2_2, OUTPUT);
  pinMode(segment2_3, OUTPUT);
  pinMode(segment2_4, OUTPUT);
  pinMode(segment2_5, OUTPUT);
  pinMode(segment2_6, OUTPUT);
  pinMode(segment2_7, OUTPUT);
  pinMode(ground,    OUTPUT);
  allPinsLOW();

    }

void loop() 
{

  Drum_Humidity = GetHumid(22, 23);
  Drum_Temperature = GetTemp(22, 23);
  Dryer_Humidity = GetHumid(18, 19);
  Dryer_Temperature = GetTemp(18, 19);
  
  /*Dew_point = Temperature - ((100 - Humidity)/5);*/
  
  Serial.print(", Drum is Temperature % C =" + String(Drum_Temperature));
  Serial.print(", Drum Humidity % RH =" + String(Drum_Humidity));
  Serial.print(", Dryer Temperature % C =" + String(Dryer_Temperature));
  Serial.println(", Dryer Humidity % RH =" + String(Dryer_Humidity));
 
  /*Serial.println(" Dew Point % DP =" + String(Dew_point));
  updateLCD(Dew_point);*/
  delay(20);
  alternate(); 
  delay(1000);
  
    }

//Function to display Dew point on LCD
void updateLCD(int dewpoint)
{
  
  seg1 = (dewpoint/10)%10; // Extracting the first digit from dew point
  seg2 = dewpoint%10; // Extracting the second digit from dew point

  //Print First digit
  if(seg1 == 0)
  {
    digitalWrite(segment1_1, HIGH); 
    digitalWrite(segment1_2, HIGH);
    digitalWrite(segment1_3, HIGH);
    digitalWrite(segment1_4, HIGH);
    digitalWrite(segment1_5, HIGH);
    digitalWrite(segment1_6, HIGH);
    digitalWrite(segment1_7, LOW);
    digitalWrite(ground, LOW);
      
      }
      
  else if(seg1 == 1)
  {
    digitalWrite(segment1_1, LOW); 
    digitalWrite(segment1_2, HIGH);
    digitalWrite(segment1_3, HIGH);
    digitalWrite(segment1_4, LOW);
    digitalWrite(segment1_5, LOW);
    digitalWrite(segment1_6, LOW);
    digitalWrite(segment1_7, LOW);
    digitalWrite(ground, LOW);
  
      }
      
  else if(seg1 == 2)
  {
    digitalWrite(segment1_1, HIGH); 
    digitalWrite(segment1_2, HIGH);
    digitalWrite(segment1_3, LOW);
    digitalWrite(segment1_4, HIGH);
    digitalWrite(segment1_5, HIGH);
    digitalWrite(segment1_6, LOW);
    digitalWrite(segment1_7, HIGH);
    digitalWrite(ground, LOW);
      
      }
      
  else if(seg1 == 3)
  {
    digitalWrite(segment1_1, HIGH); 
    digitalWrite(segment1_2, HIGH);
    digitalWrite(segment1_3, HIGH);
    digitalWrite(segment1_4, HIGH);
    digitalWrite(segment1_5, LOW);
    digitalWrite(segment1_6, LOW);
    digitalWrite(segment1_7, HIGH);
    digitalWrite(ground, LOW);
  
      }
      
  else if(seg1 == 4)
  {
    digitalWrite(segment1_1, LOW); 
    digitalWrite(segment1_2, HIGH);
    digitalWrite(segment1_3, HIGH);
    digitalWrite(segment1_4, LOW);
    digitalWrite(segment1_5, LOW);
    digitalWrite(segment1_6, HIGH);
    digitalWrite(segment1_7, HIGH);
    digitalWrite(ground, LOW);
      
      }
      
  else if(seg1 == 5)
  {
    digitalWrite(segment1_1, HIGH); 
    digitalWrite(segment1_2, LOW);
    digitalWrite(segment1_3, HIGH);
    digitalWrite(segment1_4, HIGH);
    digitalWrite(segment1_5, LOW);
    digitalWrite(segment1_6, HIGH);
    digitalWrite(segment1_7, HIGH);
    digitalWrite(ground, LOW);
  
  }
  
  else if(seg1 == 6)
  {
    digitalWrite(segment1_1, HIGH); 
    digitalWrite(segment1_2, LOW);
    digitalWrite(segment1_3, HIGH);
    digitalWrite(segment1_4, HIGH);
    digitalWrite(segment1_5, HIGH);
    digitalWrite(segment1_6, HIGH);
    digitalWrite(segment1_7, HIGH);
    digitalWrite(ground, LOW);
      
      }
      
  else if(seg1 == 7)
  {
    digitalWrite(segment1_1, HIGH); 
    digitalWrite(segment1_2, HIGH);
    digitalWrite(segment1_3, HIGH);
    digitalWrite(segment1_4, LOW);
    digitalWrite(segment1_5, LOW);
    digitalWrite(segment1_6, LOW);
    digitalWrite(segment1_7, LOW);
    digitalWrite(ground, LOW);
  
      }
      
  else if(seg1 == 8)
  {
    digitalWrite(segment1_1, HIGH); 
    digitalWrite(segment1_2, HIGH);
    digitalWrite(segment1_3, HIGH);
    digitalWrite(segment1_4, HIGH);
    digitalWrite(segment1_5, HIGH);
    digitalWrite(segment1_6, HIGH);
    digitalWrite(segment1_7, HIGH);
    digitalWrite(ground, LOW);
      
      }
      
  else if(seg1 == 9)
  {
    digitalWrite(segment1_1, HIGH); 
    digitalWrite(segment1_2, HIGH);
    digitalWrite(segment1_3, HIGH);
    digitalWrite(segment1_4, HIGH);
    digitalWrite(segment1_5, LOW);
    digitalWrite(segment1_6, HIGH);
    digitalWrite(segment1_7, HIGH);
    digitalWrite(ground, LOW);
      
      }

  //Print Second digit
  if(seg2 == 0)
  {
    digitalWrite(segment2_1, HIGH); 
    digitalWrite(segment2_2, HIGH);
    digitalWrite(segment2_3, HIGH);
    digitalWrite(segment2_4, HIGH);
    digitalWrite(segment2_5, HIGH);
    digitalWrite(segment2_6, HIGH);
    digitalWrite(segment2_7, LOW);
    digitalWrite(ground, LOW);
      
      }
      
  else if(seg2 == 1)
  {
    digitalWrite(segment2_1, LOW); 
    digitalWrite(segment2_2, HIGH);
    digitalWrite(segment2_3, HIGH);
    digitalWrite(segment2_4, LOW);
    digitalWrite(segment2_5, LOW);
    digitalWrite(segment2_6, LOW);
    digitalWrite(segment2_7, LOW);
    digitalWrite(ground, LOW);
  
  }
  
  else if(seg2 == 2)
  {
    digitalWrite(segment2_1, HIGH); 
    digitalWrite(segment2_2, HIGH);
    digitalWrite(segment2_3, LOW);
    digitalWrite(segment2_4, HIGH);
    digitalWrite(segment2_5, HIGH);
    digitalWrite(segment2_6, LOW);
    digitalWrite(segment2_7, HIGH);
    digitalWrite(ground, LOW);
      
      }
      
  else if(seg2 == 3)
  {
    digitalWrite(segment2_1, HIGH); 
    digitalWrite(segment2_2, HIGH);
    digitalWrite(segment2_3, HIGH);
    digitalWrite(segment2_4, HIGH);
    digitalWrite(segment2_5, LOW);
    digitalWrite(segment2_6, LOW);
    digitalWrite(segment2_7, HIGH);
    digitalWrite(ground, LOW);
      
      } 
      
  else if(seg2 == 4)
  {
    digitalWrite(segment2_1, LOW); 
    digitalWrite(segment2_2, HIGH);
    digitalWrite(segment2_3, HIGH);
    digitalWrite(segment2_4, LOW);
    digitalWrite(segment2_5, LOW);
    digitalWrite(segment2_6, HIGH);
    digitalWrite(segment2_7, HIGH);
    digitalWrite(ground, LOW);
      
      }
      
  else if(seg2 == 5)
  {
    digitalWrite(segment2_1, HIGH); 
    digitalWrite(segment2_2, LOW);
    digitalWrite(segment2_3, HIGH);
    digitalWrite(segment2_4, HIGH);
    digitalWrite(segment2_5, LOW);
    digitalWrite(segment2_6, HIGH);
    digitalWrite(segment2_7, HIGH);
    digitalWrite(ground, LOW);
      
      }
  else if(seg2 == 6)
  {
    digitalWrite(segment2_1, HIGH); 
    digitalWrite(segment2_2, LOW);
    digitalWrite(segment2_3, HIGH);
    digitalWrite(segment2_4, HIGH);
    digitalWrite(segment2_5, HIGH);
    digitalWrite(segment2_6, HIGH);
    digitalWrite(segment2_7, HIGH);
    digitalWrite(ground, LOW);
      
      } 
      
  else if(seg2 == 7)
  {
    digitalWrite(segment2_1, HIGH); 
    digitalWrite(segment2_2, HIGH);
    digitalWrite(segment2_3, HIGH);
    digitalWrite(segment2_4, LOW);
    digitalWrite(segment2_5, LOW);
    digitalWrite(segment2_6, LOW);
    digitalWrite(segment2_7, LOW);
    digitalWrite(ground, LOW);
      
      }
      
  else if(seg2 == 8)
  {
    digitalWrite(segment2_1, HIGH); 
    digitalWrite(segment2_2, HIGH);
    digitalWrite(segment2_3, HIGH);
    digitalWrite(segment2_4, HIGH);
    digitalWrite(segment2_5, HIGH);
    digitalWrite(segment2_6, HIGH);
    digitalWrite(segment2_7, HIGH);
    digitalWrite(ground, LOW);
      
      } 
      
  else if(seg2 == 9)
  {
    digitalWrite(segment2_1, HIGH); 
    digitalWrite(segment2_2, HIGH);
    digitalWrite(segment2_3, HIGH);
    digitalWrite(segment2_4, HIGH);
    digitalWrite(segment2_5, LOW);
    digitalWrite(segment2_6, HIGH);
    digitalWrite(segment2_7, HIGH);
    digitalWrite(ground, LOW);
      
      }
        
    }

void alternate()
{
   digitalWrite(segment1_1, LOW); 
   digitalWrite(segment1_2, LOW);
   digitalWrite(segment1_3, LOW);
   digitalWrite(segment1_4, LOW);
   digitalWrite(segment1_5, LOW);
   digitalWrite(segment1_6, LOW);
   digitalWrite(segment1_7, LOW);
   digitalWrite(segment2_1, LOW); 
   digitalWrite(segment2_2, LOW);
   digitalWrite(segment2_3, LOW);
   digitalWrite(segment2_4, LOW);
   digitalWrite(segment2_5, LOW);
   digitalWrite(segment2_6, LOW);
   digitalWrite(segment2_7, LOW);
   digitalWrite(ground, HIGH);
}

void allPinsLOW()
{
   digitalWrite(segment1_1, LOW); 
   digitalWrite(segment1_2, LOW);
   digitalWrite(segment1_3, LOW);
   digitalWrite(segment1_4, LOW);
   digitalWrite(segment1_5, LOW);
   digitalWrite(segment1_6, LOW);
   digitalWrite(segment1_7, LOW);
   digitalWrite(segment2_1, LOW); 
   digitalWrite(segment2_2, LOW);
   digitalWrite(segment2_3, LOW);
   digitalWrite(segment2_4, LOW);
   digitalWrite(segment2_5, LOW);
   digitalWrite(segment2_6, LOW);
   digitalWrite(segment2_7, LOW);
   digitalWrite(ground, LOW);
}
