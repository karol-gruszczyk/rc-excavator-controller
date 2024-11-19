
#include <Wire.h>
#include <Adafruit_MCP4725.h>

#define SERVO_1_PIN 3
#define SERVO_2_PIN 3
#define THROTTLE_MODE_PIN 6
#define THROTTLE_PIN A0

#define THERMISTOR_PIN A1
#define THERMISTOR_NOMINAL 10000   
#define TEMPERATURE_NOMINAL 25
#define B_COEFFICIENT 3950
#define THERMISTOR_SERIES_RESISTOR 10000
#define THERMISTOR_NUM_SAMPLES 5
#define THERMISTOR_MIN_TEMP 0
#define THERMISTOR_MAX_TEMP 100

// NANO PWM PINS: 3, 5, 6, 9, 10, 11
#define RADIATOR_FAN_PIN 3  // PWM
#define RADIATOR_FAN_OFF_TEMP 25  // 45
#define RADIATOR_FAN_ON_TEMP 30  // 55

#define DAC_RESULOTION (9)

#define SERVO_PULSE_RANGE 500

Adafruit_MCP4725 dac;

volatile long servo_pulse[] = {0, 0, 0, 0, 0, 0, 0};
volatile long servo_start_timer[] = {0, 0, 0, 0, 0, 0, 0};
volatile long servo_end_timer[] = {0, 0, 0, 0, 0, 0, 0};

//bool throttle_mode = THROTTLE_MODE_AUTO;
float motor_throttle = 0; // 0.0 - 1.0

float oil_temperature = 0.0;
float thermistor_reading = 0.0;
float fan_speed = 0.0; // 0.0<>1.0

#define ERROR_CODE_BLINK_DELAY 250
#define ERROR_CODE_CYCLE_DELAY 1000
#define ERROR_CODE_CYCLE_RESTART_DELAY 3000

#define ERROR_CODE_BITS 8
#define ERROR_CODE_THROTTLE_INIT_VALUE 0  // before accepting any input value, it needs to read 0 first
#define ERROR_CODE_THERMISTOR_RANGE 1

uint8_t error_codes_set = 0;  // each bit is an individual error code
uint8_t current_error_bit = 0;
uint8_t current_error_blink_counter = 0;
volatile long error_timer = 0;


void setup() {
    Serial.begin(9600);
    Serial.println("Init");

//  pinMode(SERVO_1_PIN, INPUT_PULLUP);
//  pinMode(SERVO_2_PIN, INPUT_PULLUP);

  pinMode(THROTTLE_PIN, INPUT);
//  pinMode(THERMISTOR_PIN, INPUT);
//  pinMode(RADIATOR_FAN_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

//  attachInterrupt(digitalPinToInterrupt(SERVO_1_PIN), pulse_timer_1_rising, HIGH);
//  attachInterrupt(digitalPinToInterrupt(SERVO_1_PIN), pulse_timer_1_falling, FALLING);

  dac.begin(0x60);
//    dac.setVoltage(0, false);

  bitSet(error_codes_set, ERROR_CODE_THROTTLE_INIT_VALUE);
  Serial.println("Done");
}

// the loop function runs over and over again forever
void loop() {

//    Serial.println(motor_throttle * 100);
//    Serial.print(" ");
//    Serial.println(voltage);

    throttle_callback();
    thermostat_callback();
    error_callback();
    delay(500);
}

void throttle_callback() {
//    throttle_mode = digitalRead(THROTTLE_MODE_PIN) == HIGH;

//    if (throttle_mode == THROTTLE_MODE_CONSTANT) {
      motor_throttle = analogRead(THROTTLE_PIN) / 1023.0;
      
      if (bitRead(error_codes_set, ERROR_CODE_THROTTLE_INIT_VALUE)) {  // safe startup
          if (motor_throttle == 0.0) {
              bitClear(error_codes_set, ERROR_CODE_THROTTLE_INIT_VALUE);
          } else {
              motor_throttle = 0.0;
          }
      }
//    } else {
//      motor_throttle = min(abs(servo_pulse[0] - 1500) / SERVO_PULSE_RANGE, 1.0);
//    }

    dac.setVoltage(min(motor_throttle * 4095, 4095), false);

    Serial.print("THROTTLE: ");
    Serial.println(motor_throttle);
}

void thermostat_callback() {
    uint8_t i;
    
    thermistor_reading = 0.0;
    for (i = 0; i < THERMISTOR_NUM_SAMPLES; i++) {
        thermistor_reading += analogRead(THERMISTOR_PIN);
        delay(10);
    }
    thermistor_reading /= THERMISTOR_NUM_SAMPLES;

    thermistor_reading = (1023 / thermistor_reading)  - 1;     // (1023/ADC - 1) 
    thermistor_reading = THERMISTOR_SERIES_RESISTOR / thermistor_reading;

    float steinhart;
    steinhart = thermistor_reading / THERMISTOR_NOMINAL;     // (R/Ro)
    steinhart = log(steinhart);                  // ln(R/Ro)
    steinhart /= B_COEFFICIENT;                   // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15); // + (1/To)
    oil_temperature = 1.0 / steinhart - 273.15;

    bitWrite(error_codes_set, ERROR_CODE_THERMISTOR_RANGE, oil_temperature < THERMISTOR_MIN_TEMP || oil_temperature > THERMISTOR_MAX_TEMP);

    fan_speed = min(1.0, max(0.0, oil_temperature - RADIATOR_FAN_OFF_TEMP) / (RADIATOR_FAN_ON_TEMP - RADIATOR_FAN_OFF_TEMP));
    analogWrite(RADIATOR_FAN_PIN, fan_speed * 255.0);

    Serial.print("FAN SPEED: ");
    Serial.print(fan_speed);
    Serial.print(" OIL TEMP: ");
    Serial.println(oil_temperature);
    
}

void error_callback() {
    int timer_interval = millis() - error_timer;

    if (
        (current_error_bit == 0 && current_error_blink_counter == 0 && timer_interval < ERROR_CODE_CYCLE_RESTART_DELAY)  // error cycle restarts
        || (current_error_blink_counter == 0 && timer_interval < ERROR_CODE_CYCLE_DELAY)  // done reporting error
        || (current_error_blink_counter > 0 && timer_interval < ERROR_CODE_BLINK_DELAY)  // reporting error
    ) {
        return;  // sleep
    }

    error_timer = millis();

    Serial.print("ERROR BITMAP: ");
    Serial.println(error_codes_set);
  
    while (true) {
        if (current_error_blink_counter > 0 || bitRead(error_codes_set, current_error_bit)) {
//            Serial.print("BIT SET: ");
//            Serial.print(current_error_bit);
//            Serial.print(" BLINK STEP: ");
//            Serial.print(current_error_blink_counter);
            if (current_error_blink_counter % 2 == 0) {
                digitalWrite(LED_BUILTIN, HIGH);
//                Serial.println("HIGH");
            } else {
                digitalWrite(LED_BUILTIN, LOW);
//                Serial.println("LOW");
            }
            if (++current_error_blink_counter >= (current_error_bit + 1) * 2) {
                current_error_blink_counter = 0;
                current_error_bit++;
            }
            break;
        }

        if (++current_error_bit >= ERROR_CODE_BITS) {
            current_error_bit = 0;
            break;
        }
    }
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
