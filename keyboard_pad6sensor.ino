#include <base.h>
#include <DirectIO.h>
#include <ports.h>
#include <Keyboard.h>

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// set prescale to 16

// Measure the voltage at 5V and resistance of your 3.3k resistor, and enter
// their value's below:
const float VCC = 3.3; // Measured voltage of Ardunio line
const float R_DIV = 3300.00; // Measured resistance of 3.3k resistor

// Clockwise starting at top_left
// {top_left, top_right, right_top, right_bottom, bottom_right, bottom_left, left_bottom, left_top}
int sensitivities[6] = {999, 999, 999, 999, 999, 999};
bool triggers[4] = {false, false, false, false};

String valueToAdjust = "";
bool listenForValue;

String valueToSet = "";

int numberInSequence = 0;
int serialLength = 0;

bool debug = false;

char up = 'w';
char right = 'd';
char down = 's';
char left = 'a';

void setup() 
{
  Serial.begin(500000);
  
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
  
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
}

void loop() 
{
  serialLength = Serial.available();
  if(serialLength) {
    valueToSet = Serial.readStringUntil(',');
    Serial.println(valueToSet);
    if(valueToSet == "sens") {
      printSens();
      return;
    }
    if (valueToSet == "debug") {
      debug = !debug;
      return;
    }
    
    sensitivities[numberInSequence] = valueToSet.toInt();
    
    if (numberInSequence == 5) {
      printSens();
      numberInSequence = 0;
    } else {
      numberInSequence++;
    }
  } else {
    float topLPressure = readPressure(A0);
    float topRPressure = readPressure(A1);
    bool topTriggered = (topLPressure > sensitivities[0] || topRPressure > sensitivities[1]) ? true : false;

    float rightPressure = readPressure(A2);
    bool rightTriggered = rightPressure > sensitivities[2] ? true : false;
 
    float downRPressure = readPressure(A3);
    float downLPressure = readPressure(A4);
    bool downTriggered = (downRPressure > sensitivities[3] || downLPressure > sensitivities[4]) ? true : false;

    float leftPressure = readPressure(A5);
    bool leftTriggered = (leftPressure > sensitivities[5]) ? true : false;

    if (debug) {
      Serial.print("Pressure: ");
      Serial.print("topleft: "    +  String(topLPressure) + (topTriggered ? "(X) " : " "));
      Serial.print("topright: "    +  String(topRPressure) + (topTriggered ? "(X) " : " "));
      Serial.print("right: "   +  String(rightPressure) + (rightTriggered ? "(X) " : " "));
      Serial.print("downright: "   +  String(downRPressure) + (downTriggered ? "(X) " : " "));
      Serial.print("downleft: "   +  String(downLPressure) + (downTriggered ? "(X) " : " "));
      Serial.print("left: "   +  String(leftPressure) + (leftTriggered ? "(X) " : " "));
      Serial.println();
    }
    if (topTriggered && !triggers[0]) {
      triggers[0] = true;
      Keyboard.press(up);
    } else if (!topTriggered && triggers[0]) {
      triggers[0] = false;
      Keyboard.release(up);
    }

    if (rightTriggered && !triggers[1]) {
      triggers[1] = true;
      Keyboard.press(right);
    } else if (!rightTriggered && triggers[1]) {
      triggers[1] = false;
      Keyboard.release(right);
    }

    if (downTriggered && !triggers[2]) {
      triggers[2] = true;
      Keyboard.press(down);
    } else if (!downTriggered && triggers[2]){
      triggers[2] = false;
      Keyboard.release(down);
    }

    if (leftTriggered && !triggers[3]) {
      triggers[3] = true;
      Keyboard.press(left);
    } else if (!leftTriggered && triggers[3]){
      triggers[3] = false;
      Keyboard.release(left);
    }
    
  }
  
}

void printSens() {
  Serial.println();
  Serial.println("Sensitivities:");
  Serial.println("topleft: " + String(sensitivities[0]));
  Serial.println("topright: " + String(sensitivities[1]));
  Serial.println("right: " + String(sensitivities[2]));
  Serial.println("bottomright: " + String(sensitivities[3]));
  Serial.println("bottomleft: " + String(sensitivities[4]));
  Serial.println("left: " + String(sensitivities[5]));
  Serial.println();
}

float readPressure(int pin) {
  int fsrADC = analogRead(pin);
  // If the FSR has no pressure, the resistance will be
  // near infinite. So the voltage should be near 0.
  if (fsrADC != 0) // If the analog reading is non-zero
  {
    // Use ADC reading to calculate voltage:
    float fsrV = fsrADC * VCC / 1023.0;
    // Use voltage and static resistor value to 
    // calculate FSR resistance:
    float fsrR = R_DIV * (VCC / fsrV - 1.0);
    // Guesstimate force based on slopes in figure 3 of
    // FSR datasheet:
    float force;
    float fsrG = 1.0 / fsrR; // Calculate conductance
    // Break parabolic curve down into two linear slopes:
    if (fsrR <= 600) 
      force = (fsrG - 0.00075) / 0.00000032639;
    else
      force =  fsrG / 0.000000642857;

    return force;
  }
  else
  {
    return 0;
    // No pressure detected
  }
}

float calcPressure(int fsrADC) {
//  int fsrADC = analogRead(pin);
  // If the FSR has no pressure, the resistance will be
  // near infinite. So the voltage should be near 0.
  if (fsrADC != 0) // If the analog reading is non-zero
  {
    // Use ADC reading to calculate voltage:
    float fsrV = fsrADC * VCC / 1023.0;
    // Use voltage and static resistor value to 
    // calculate FSR resistance:
    float fsrR = R_DIV * (VCC / fsrV - 1.0);
    // Guesstimate force based on slopes in figure 3 of
    // FSR datasheet:
    float force;
    float fsrG = 1.0 / fsrR; // Calculate conductance
    // Break parabolic curve down into two linear slopes:
    if (fsrR <= 600) 
      force = (fsrG - 0.00075) / 0.00000032639;
    else
      force =  fsrG / 0.000000642857;

    return force;
  }
  else
  {
    return 0;
    // No pressure detected
  }
}
