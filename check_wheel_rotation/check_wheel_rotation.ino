#include <SoftwareSerial.h>

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

void setup() {
  Serial.begin(9600); // Initialize the hardware serial port for debugging
  // mySerial.begin(115200); // Initialize the software serial port
}


void loop() {
  //gpt-bro///////////////////////////////////////////////////////
  while (Serial.available()) {
    char data = Serial.read();

    // Input format: <|255,255|>
    
    if (data == '<' && Serial.peek() == '|') {
      // Start recording
      recording = true;
      inputString = "";
      Serial.read(); // consume '|'
    } else if (recording && data == '|' && Serial.peek() == '>') {
      // Stop recording
      recording = false;
      Serial.read(); // consume '>'
      // Process the recorded data
      processInputString(inputString);
    } else if (recording) {
      // Record data
      inputString += data;
    }

    // Serial.println(data);
  }

  delay(50);
  //////////////////////////////////////////////////////////////////
drive();

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


void processInputString(String input) {
  int commaIndex = input.indexOf(',');
  
  if (commaIndex != -1) {
    String xString = input.substring(0, commaIndex);
    String yString = input.substring(commaIndex + 1);

    left_track_val = xString.toInt();
    right_track_val = yString.toInt();

    Serial.print("left : ");
    Serial.println(left_track_val);
    Serial.print("right : ");
    Serial.println(right_track_val);
  } else 
  {
    Serial.println("Invalid input format");
  }
}