// ProportionalControl.pde
// -*- mode: C++ -*-
//
// Make a single stepper follow the analog value read from a pot or whatever
// The stepper will move at a constant speed to each newly set posiiton, 
// depending on the value of the pot.
//
// Copyright (C) 2012 Mike McCauley
// $Id: ProportionalControl.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>

// Define a stepper and the pins it will use
// step pin is 2, direction pin is 3
AccelStepper stepper(1,2,3); 

// This defines the analog input pin for reading the control voltage
// Tested with a 10k linear pot between 5v and GND
#define ANALOG_IN A0

int last_in = 0;

void setup()
{  
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(1000);
  Serial.begin(115200);
}

void loop()
{
  // Read new position
  int analog_in = analogRead(ANALOG_IN);
  analog_in = 0.2*analog_in + last_in*0.8;
  stepper.moveTo(analog_in*1.2);
  stepper.setSpeed(5000);
  //stepper.run();
  stepper.runSpeedToPosition();
  Serial.println(analog_in);

  last_in = analog_in;
}
