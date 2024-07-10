
#include <Wire.h>
#include <Adafruit_MCP4725.h>

#define SERVO_1_PIN 3
#define SERVO_2_PIN 3
#define THROTTLE_MODE_PIN 6
#define THROTTLE_PIN 7
#define DAC_RESULOTION (9)

#define SERVO_PULSE_RANGE 500

#define THROTTLE_MODE_AUTO false
#define THROTTLE_MODE_CONSTANT true

#define THERMOSTAT_OPEN_TEMP 80.0;
#define THERMOSTAT_FAN_TEMP 100.0;

Adafruit_MCP4725 dac;

volatile long servo_pulse[] = {0, 0, 0, 0, 0, 0, 0};
volatile long servo_start_timer[] = {0, 0, 0, 0, 0, 0, 0};
volatile long servo_end_timer[] = {0, 0, 0, 0, 0, 0, 0};
bool throttle_mode = THROTTLE_MODE_AUTO;
float motor_throttle = 0; // 0.0 - 1.0

float oil_temperature = 0.0;

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
//  dac.setVoltage(0, false);
  
//  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SERVO_1_PIN, INPUT_PULLUP);
//  pinMode(SERVO_2_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  Serial.println("Sig");

  attachInterrupt(digitalPinToInterrupt(SERVO_1_PIN), pulse_timer_1_rising, HIGH);
//  attachInterrupt(digitalPinToInterrupt(SERVO_1_PIN), pulse_timer_1_falling, FALLING);
//  attachInterrupt(digitalPinToInterrupt(SERVO_2_PIN), pulse_timer_2, CHANGE);

  dac.begin(0x60);
}

// the loop function runs over and over again forever
void loop() {
    throttle_mode = digitalRead(THROTTLE_MODE_PIN) == HIGH;

    if (throttle_mode == THROTTLE_MODE_CONSTANT) {
      motor_throttle = analogRead(THROTTLE_PIN) / 255;
    } else {
      motor_throttle = min(abs(servo_pulse[0] - 1500) / SERVO_PULSE_RANGE, 1.0);
    }

    dac.setVoltage(min(motor_throttle * 4095, 4095), false);


    Serial.println(servo_pulse[0]);
//    Serial.print(" ");
//    Serial.println(voltage);

    thermostat();
}

void thermostat() {
//    oil_temperature = ...

//  solenoid_on = (oil_temperature >= THERMOSTAT_OPEN_TEMP);

//  fan_on = (oil_temperature >= THERMOSTAT_FAN_TEMP);
    
}

bool pulse_timer_rising(int pin) {
  servo_start_timer[pin] = micros();
  return servo_start_timer[pin] - servo_end_timer[pin] > 20000;
}

void pulse_timer_falling(int pin) {
  servo_end_timer[pin] = micros();
  if (servo_end_timer[pin] - servo_start_timer[pin] >= 790 && servo_end_timer[pin] - servo_start_timer[pin] <= 2210) {
    servo_pulse[pin] = (servo_pulse[pin] * 4 + servo_end_timer[pin] - servo_start_timer[pin]) / 5;
  }
}


void pulse_timer_1_rising() {
  if (pulse_timer_rising(0)) {
    attachInterrupt(digitalPinToInterrupt(SERVO_1_PIN), pulse_timer_1_falling, LOW);
  }
}

void pulse_timer_1_falling() {
  pulse_timer_falling(0);
  attachInterrupt(digitalPinToInterrupt(SERVO_1_PIN), pulse_timer_1_rising, HIGH);
}

//void pulse_timer_2() {
//  pulse_timer(1);
//}
