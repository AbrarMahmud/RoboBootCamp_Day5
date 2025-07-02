// Pin Assignment-------

// iR array ----- Arduino
// ----------------------
// Vcc      ----- 5V
// GND      ----- GND
// D1       ----- A0
// D2       ----- A1
// D3       ----- A3
// D4       ----- A4
// D6       ----- A5

// Motor Driver - Arduino
// -----------------------
// ENB      ----- 11
// IN3      ----- 12
// IN4      ----- 13

// ENA      ----- 10
// IN1      ----- 8
// IN2      ----- 9
//------------------------

// #include <SoftwareSerial.h>
#include <QTRSensors.h>
#include <PID_v1.h>

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


float left_track_val = 0;
float right_track_val = 0;
const int left_track_val_max=105,left_track_val_min=60;
const int right_track_val_max=105,right_track_val_min=60;
const float wheelbase = 0.13; //m
float max_linear_velo_x = 1.0 ,max_angular_velo_z = 5.5; // linear_velo : 1 m/s and angular_velo : 2 rad/s

double Setpoint, PID_Input, PID_Output;
//Specify the links and initial tuning parameters
// better performing values :
// kp , ki , kd
// 1.85015,0.1,0.0
double Kp=1.35015, Ki=0.1, Kd=0.02;
PID PID_controller(&PID_Input, &PID_Output, &Setpoint,Kp, Ki, Kd,P_ON_E, DIRECT); //P_ON_M specifies that Proportional on Measurement be used --  make the output move more smoothly when the setpoint is changed.
                                                            //P_ON_E (Proportional on Error) is the default behavior
// Worked like am on-off control
// PID PID_controller(&PID_Input, &PID_Output, &Setpoint,3.15,0,0.0,P_ON_E, DIRECT); //P_ON_M specifies that Proportional on Measurement be used --  make the output move more smoothly when the setpoint is changed.
//                                                             //P_ON_E (Proportional on Error) is the default behavior


QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

void setup() {
  Serial.begin(9600); // Initialize the hardware serial port for debugging
  
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
  //  Call calibrate once to allocate the calibration arrays

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


  Setpoint = 1800;  // QTR_sensor mid value

  //turn the PID on
  PID_controller.SetSampleTime(9.5);
  PID_controller.SetOutputLimits(-2500, 2500);// -100---cw turn  and 100--ccw 
                                            // rad/s of rotational velocity
  PID_controller.SetMode(AUTOMATIC);
}

float i = 0.0;
void loop() {
  //Line Follow///////////////////////////////////////////////////////
  uint16_t position = qtr.readLineBlack(sensorValues);
  // uint16_t position = qtr.readLineWhite(sensorValues);
  PID_Input = double(position);
  PID_controller.Compute();
  // Serial.print(PID_Input);
  // Serial.print(",");
  // Serial.print(PID_Output);
  float mapped_output = piece_wise_functional_map(PID_Output,0.0,2500,0,max_angular_velo_z);
  // Serial.print(",");
  // Serial.println(mapped_output);
  drive(-1.0*mapped_output,0.3);
  //////////////////////////////////////////////////////////////////
  //  
  // drive(2*cos(2*3.415*i),0.1);
  // i = i + 0.01;
  delay(10);
}

void drive(float angular_velo_z,float linear_velo_x)
{
  // -ve -- CW (right turn)
  // +ve -- CCW (left turn)
  float speed_req_left = linear_velo_x - angular_velo_z*(wheelbase/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
  float speed_req_right = linear_velo_x + angular_velo_z*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds

  float max_wheel_velo = linear_velo_x + max_angular_velo_z*(wheelbase/2);
  // Serial.println(max_wheel_velo);
  // Serial.print(",");
  left_track_val = int(piece_wise_functional_map(speed_req_left,0.0,max_wheel_velo,left_track_val_min,left_track_val_max));
  right_track_val = int(piece_wise_functional_map(speed_req_right,0.0,max_wheel_velo,right_track_val_min,right_track_val_max));
  
  Serial.print(angular_velo_z);
  Serial.print(",");
  Serial.print(speed_req_left);
  Serial.print(",");
  Serial.print(speed_req_right);
  Serial.print(",");
  Serial.print(left_track_val);
  Serial.print(",");
  Serial.println(right_track_val);


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


float piece_wise_functional_map(float in_val,float abs_x_min,float abs_x_max,int abs_y_min,int abs_y_max)
{
  float slope = (abs_y_max-abs_y_min)/(abs_x_max-abs_x_min)*1.0;
  float tmp_map = slope*1.0*(abs(in_val)-abs_x_min) + abs_y_min;

  // Built-in map() will not work
  // float tmp_map = map(abs(in_val),abs_x_min,abs_x_max,abs_y_min,abs_y_max);
  
  return ((in_val<0.0)?(-1.0*tmp_map):(1.0*tmp_map));
}