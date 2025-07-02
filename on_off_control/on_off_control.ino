// #include <SoftwareSerial.h>
#include <QTRSensors.h>

//MOTOR Right
const int IN_3=12;
const int IN_4=13;
const int EN_left=11; //speed control
const int offset_left = 0;

//MOTOR Left
const int IN_1=8;
const int IN_2=9;
const int EN_right=10; //speed control
const int offset_right = 0;


int left_track_val = 0;
int right_track_val = 0;
String inputString = "";
boolean recording = false;

// SoftwareSerial mySerial(2, 3); // RX, TX pins for SoftwareSerial
QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

void setup() {
  Serial.begin(9600); // Initialize the hardware serial port for debugging
  // mySerial.begin(115200); // Initialize the software serial port
  // configure the sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5}, SensorCount);
  //qtr.setEmitterPin(2);

  delay(500);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.

  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  delay(1000);
}


void loop() {
  //Line Follow///////////////////////////////////////////////////////
  uint16_t position = qtr.readLineBlack(sensorValues);
  if(position>2200)
  {
    left_track_val = 105;//60
    right_track_val = -80;//-45
    // delay(50);
  }else if(position<2100){
    left_track_val = -80;
    right_track_val = 105;
    // delay(50);
  }else{
    left_track_val = 100;
    right_track_val = 100;
  }
  //////////////////////////////////////////////////////////////////
drive();
delay(5);
}

void drive()
{
  //if left_track_val == (-ve) ,then the left motor will revers its spinning
  //so left motor reverse   IN_3 --> LOW
  //                        IN_4 --> HIGH
  digitalWrite(IN_3,((left_track_val<0)?LOW:HIGH));
  digitalWrite(IN_4,((left_track_val<0)?HIGH:LOW));
  analogWrite(EN_left,abs(left_track_val));


  //if right_track_val == (-ve) ,then the right motor will revers its spinning
  //so lright motor reverse   IN_2 --> LOW
  //                          IN_1 --> HIGH
  digitalWrite(IN_2,((right_track_val<0)?LOW:HIGH));
  digitalWrite(IN_1,((right_track_val<0)?HIGH:LOW));
  analogWrite(EN_right,abs(right_track_val));
}
