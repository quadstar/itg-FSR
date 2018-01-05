#include <base.h>
#include <DirectIO.h>
#include <ports.h>

 /******************************************************************************
Force_Sensitive_Resistor_Example.ino
Example sketch for SparkFun's force sensitive resistors
  (https://www.sparkfun.com/products/9375)
Jim Lindblom @ SparkFun Electronics
April 28, 2016

Create a voltage divider circuit combining an FSR with a 3.3k resistor.
- The resistor should connect from A0 to GND.
- The FSR should connect from A0 to 3.3V
As the resistance of the FSR decreases (meaning an increase in pressure), the
voltage at A0 should increase.

Development environment specifics:
Arduino 1.6.7
******************************************************************************/

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
const float VCC = 3.3; // Measured voltage of Ardunio 3,3V line
const float R_DIV = 3300.00; // Measured resistance of 3.3k resistor

// Clockwise starting at top_left
// {top_left, top_right, right_top, right_bottom, bottom_right, bottom_left, left_bottom, left_top}
int sensitivities[8] = {0, 0, 0, 0, 0, 0, 0, 0};

String valueToAdjust = "";
bool listenForValue;

String valueToSet = "";

int numberInSequence = 0;
int serialLength = 0;

bool debug = false;

//Input<54> up_left;
//Input<55> up_right;
//Input<56> left;
//
Output<62> up;
Output<63> right;
Output<64> down;
Output<65> left;
//OutputPort<PORT_K, 0, 1> piuio;

void setup() 
{
  Serial.begin(500000);
  
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
  
//  piuio = 0x00;
  
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);

  up = HIGH;
  right = HIGH;
  down = HIGH;
  left = HIGH;
//  pinMode(A8, OUTPUT);
//  digitalWrite(A8, HIGH);
//  pinMode(A9, OUTPUT);
//  digitalWrite(A9, HIGH);
//  pinMode(A10, OUTPUT);
//  digitalWrite(A10, HIGH);
//  pinMode(A11, OUTPUT);
//  digitalWrite(A11, HIGH);
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
    
    if (numberInSequence == 7) {
      printSens();
      numberInSequence = 0;
    } else {
      numberInSequence++;
    }
  } else {
    float topLeftPressure = readPressure(A0);
    float topRightPressure = readPressure(A1);
    bool topLeftTriggered = (topLeftPressure > sensitivities[0]) ? true : false;
    bool topRightTriggered = (topRightPressure > sensitivities[1]) ? true : false;

    float rightTopPressure = readPressure(A2);
    float rightBottomPressure = readPressure(A3);
    bool rightTriggered = (rightTopPressure > sensitivities[2] || rightBottomPressure > sensitivities[3]) ? true : false;
 
    float downRightPressure = readPressure(A4);
    float downLeftPressure = readPressure(A5);
    bool downTriggered = (downRightPressure > sensitivities[4] || downLeftPressure > sensitivities[5]) ? true : false;

    float leftBottomPressure = readPressure(A6);
    float leftTopPressure = readPressure(A7);
    bool leftTriggered = (leftBottomPressure > sensitivities[6] || leftTopPressure > sensitivities[7]) ? true : false;

    if (debug) {
      Serial.print("Pressure: ");
      Serial.print("top_left: "    +  String(topLeftPressure) + (topLeftTriggered ? "(X) " : " "));
      Serial.print("top_right: "   +  String(topRightPressure) + (topRightTriggered ? "(X) " : " "));
      Serial.print("right_top: "   +  String(rightTopPressure) + (rightTriggered ? "(X) " : " "));
      Serial.print("right_bottom: "   +  String(rightBottomPressure) + (rightTriggered ? "(X) " : " "));
      Serial.print("down_right: "   +  String(downRightPressure) + (downTriggered ? "(X) " : " "));
      Serial.print("down_left: "   +  String(downLeftPressure) + (downTriggered ? "(X) " : " "));
      Serial.print("left_bottom: "   +  String(leftBottomPressure) + (leftTriggered ? "(X) " : " "));
      Serial.print("left_top: "   +  String(leftTopPressure) + (leftTriggered ? "(X) " : " "));
      Serial.println();
    }
    if (topLeftTriggered || topRightTriggered) {
      up = LOW;
    } else {
      up = HIGH;
    }

    if (rightTriggered) {
      right = LOW;
    } else {
      right = HIGH;
    }

    if (downTriggered) {
      down = LOW;
    } else {
      down = HIGH;
    }

    if (leftTriggered) {
      left = LOW;
    } else {
      left = HIGH;
    }
    
  }
  
  
}

void printSens() {
  Serial.println();
  Serial.println("Sensitivities:");
  Serial.println("top_left: " + String(sensitivities[0]));
  Serial.println("top_right: " + String(sensitivities[1]));
  Serial.println("right_top: " + String(sensitivities[2]));
  Serial.println("right_bottom: " + String(sensitivities[3]));
  Serial.println("bottom_right: " + String(sensitivities[4]));
  Serial.println("bottom_left: " + String(sensitivities[5]));
  Serial.println("left_bottom: " + String(sensitivities[6]));
  Serial.println("left_top: " + String(sensitivities[7]));
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
