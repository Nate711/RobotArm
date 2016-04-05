#include <Servo.h>
#include <AccelStepper.h>
//#include <PWMServo.h>

//Servo pins (digital)
#define GRIPPER 5
#define WRIST 6
#define ELBOW 7
#define SHOULDER 8
#define FENCE 4

#define BASE_POT A0
#define SHOULDER_POT A1
#define ELBOW_POT A2
#define WRIST_POT A3
#define GRIPPER_POT A4

#define BASE_INCR 10 // pins to incr/decr base stepper
#define BASE_DECR 11

#define FENCE_BUTTON 9

#define MAX_SPEED 10000
#define MAX_ACCEL 5000

// ANGLES DEG
double base;
double shoulder;
double elbow;
double wrist;
double gripper;

double fence=90;

const unsigned int mem_size = 10;
unsigned int base_pot_mem[mem_size];
unsigned int base_pot_index = 0;

// step pin is 2, direction pin is 3
AccelStepper base_stepper(1,2,3);

double base_offset = 0; // changed by buttons so we can reset / manually move stepper

Servo base_servo;
Servo shoulder_servo;
Servo elbow_servo;
Servo wrist_servo;
Servo gripper_servo;

Servo fence_servo;

long last_servo_write = 0;
long last_base_offset = 0;
long last_step = 0;

bool last_button_state = false; // false = unpressed, true = pressed

int activated = 1; // not using activation button

// the setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200);

  wrist_servo.attach(WRIST);
  elbow_servo.attach(ELBOW);
  shoulder_servo.attach(SHOULDER);

  gripper_servo.attach(GRIPPER);

  fence_servo.attach(FENCE);

  base_stepper.setMaxSpeed(MAX_SPEED);
  base_stepper.setAcceleration(MAX_ACCEL);

  last_servo_write = millis();
  last_base_offset = millis();
  last_step = micros();

  pinMode(9,INPUT);

  pinMode(BASE_INCR,INPUT_PULLUP);
  pinMode(BASE_DECR,INPUT_PULLUP);
  pinMode(FENCE_BUTTON,INPUT_PULLUP);

  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
}

void readPots() {
  int baseV = analogRead(BASE_POT);
  int shoulderV = analogRead(SHOULDER_POT);
  int elbowV = analogRead(ELBOW_POT);
  int wristV = analogRead(WRIST_POT);
  int gripperV = analogRead(GRIPPER_POT);

  /*
  Serial.print(baseV);
  Serial.print('\t');
  Serial.print(shoulderV);
  Serial.print('\t');
  Serial.print(elbowV);
  Serial.print('\t');
  Serial.print(wristV);
  Serial.print('\t');
  Serial.println(gripperV);
  */

  // tack new value on to array
  base_pot_mem[base_pot_index] = baseV;
  // update index
  base_pot_index++;
  if(base_pot_index >= mem_size) {
    base_pot_index = 0;
  }
  // calc average
  baseV = average_array(base_pot_mem,mem_size);

  //baseV should center around 510
  base = map(baseV,88,931,-1000,1000) + (int)base_offset; // unsure about this mapping (stepper)
  base_stepper.moveTo(base);

  shoulder = map(shoulderV,88,940,180,0); // arm leans backwards when should be upright
  elbow = map(elbowV,88,950,180,0); // elbow too high when should be perp to first alu arm
  // jitter doesn't go away when hardcode elbow
  // elbow still jitters when connected to gripper digital output pin
  wrist = map(wristV,64,936,180,0);
  gripper = map(gripperV,0,1024,0,120); // calibrated for full open/close
}

int average_array(unsigned int *array, int size) {
  int sum = 0;
  for(int i=0; i<size; i++) {
    sum += array[i];
  }
  return sum/size;
}

void writeServos() {
  wrist_servo.write(wrist);
  elbow_servo.write(elbow);
  shoulder_servo.write(shoulder);
  base_servo.write(base);

  gripper_servo.write(gripper);

  fence_servo.write(fence);
}

// with teensy put this on a timer
void writeStepper() {
  base_stepper.run();
}

void moveFence() {
  bool new_button_state = digitalRead(FENCE_BUTTON) == LOW;
  if(!new_button_state & last_button_state) { // button is now unpressed but was pressed before
    fence = (fence == 90) ? 3  : 90; // toggle between 0 and 90
  }
  last_button_state = new_button_state;
}

void offsetBase(double offset) {
  if(digitalRead(BASE_INCR) == LOW) {
    base_offset += offset;
  }
  if(digitalRead(BASE_DECR) == LOW) {
    base_offset -= offset;
  }
}

void printAngles() { // takes 2.2 ms
  Serial.print("Angles\tBase: ");
  Serial.print(base);
  Serial.print("\tShoulder: ");
  Serial.print(shoulder);
  Serial.print("\tElbow:");
  Serial.print(elbow);
  Serial.print("\tWrist: ");
  Serial.print(wrist);
  Serial.print("\tGripper: ");
  Serial.println(gripper);
  //Serial.print('\t');
  //Serial.println(analogRead(GRIPPER_POT));

}

// WARNING: PRINTING ANYTHING TO SERIAL TOTALLY FUCKS THE SERVO TIMING WITH ARDUINO
void loop() {
  if(activated == 0 && digitalRead(9) == LOW) {
    activated = 1;
  }
  if(activated == 1) {
    // 50 HZ LOOP = max update rate of escs

    // 50 hz line up with thing
    if(millis() - last_servo_write > 20) { // TAKES 192 MICROS
      /************/
      //printAngles();
      /************/
      readPots();
      writeServos();
      moveFence();
      last_servo_write = millis();
    }
    // 30hz
    if(millis() - last_base_offset > 20) {
      offsetBase(2.0);
      last_base_offset = millis();
    }
    //As fast as possible
    if(micros() - last_step > 50) {
      writeStepper();
      last_step = micros();
    }
  }
}
