#include "LedControl.h"
#define joystick_x A0
#define kill_switch 8
LedControl lc=LedControl(7,6,5,1);

int ones;
int tens;
int hundreds;
int thousands;  
int speed_int;
int kill;
void setup() {
  Serial.begin(9600);
  pinMode(joystick_x, INPUT);
  pinMode(kill_switch, INPUT_PULLUP);
}

void loop() {
  if(Serial.available() > 0){
  int x_val = analogRead(joystick_x);
  kill = digitalRead(kill_switch);
  if(kill != HIGH){
    Serial.print('A');
    Serial.print('n');
  }
  Serial.print(x_val);
  Serial.print('n');
  delay(100);
  }
}
