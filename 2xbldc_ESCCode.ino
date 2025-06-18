/**
 * Dual‐motor voltage→velocity_openloop control on ESP32 WROOM
 *
 * - No encoder feedback: each motor spins at the commanded electrical speed.
 * - Uses fixed‐rate timing via micros() for smooth 10 kHz updates.
 * - Serial Monitor (115200 baud) to set each motor’s input voltage:
 *     Format: "<motor_id> <voltage>"
 *       1 <v> → Motor 1 uses v [V]
 *       2 <v> → Motor 2 uses v [V]
 *       3 <v> → Both motors use v [V]
 *   Example: "3 2.0" sets both motors so that v1 = v2 = 2 V input.
 *
 * Pin usage (GPIO on ESP32):
 *   Motor 1: UH=25  UL=26  VH=27  VL=14  WH=12  WL=13
 *   Motor 2: UH=32  UL=33  VH=18  VL=19  WH=21  WL=22
 */

#include <SimpleFOC.h>

// === Motor 1 pins ===
#define M1_WH 12
#define M1_WL 13
#define M1_VH 27
#define M1_VL 14
#define M1_UH 25
#define M1_UL 26

// === Motor 2 pins ===
#define M2_WH 21
#define M2_WL 22
#define M2_VH 18
#define M2_VL 19
#define M2_UH 32
#define M2_UL 33

// instantiate motors & drivers
BLDCMotor  motor1 = BLDCMotor(15);  // set your pole–pair count
BLDCDriver6PWM driver1 = BLDCDriver6PWM(M1_UH, M1_UL, M1_VH, M1_VL, M1_WH, M1_WL);

BLDCMotor  motor2 = BLDCMotor(15);  // set your pole–pair count
BLDCDriver6PWM driver2 = BLDCDriver6PWM(M2_UH, M2_UL, M2_VH, M2_VL, M2_WH, M2_WL);

// per-motor “input voltage” from Serial
float target_voltage1 = 0;
float target_voltage2 = 0;

// fixed‐rate timing for 10 kHz updates
static unsigned long t_prev = 0;
const unsigned long update_interval = 100; // µs

// time tracking for command timeout
unsigned long last_command_time = 0;
const unsigned long command_timeout = 1000000; // 1 second in microseconds

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Dual‐Motor voltage→velocity_openloop ===");
  Serial.println("Format: \"<motor_id> <voltage>\"");
  Serial.println("  1 <V> → set Motor 1 input voltage");
  Serial.println("  2 <V> → set Motor 2 input voltage");
  Serial.println("  3 <V> → set both motors’ input voltage\n");

  // --- Driver & Motor 1 init ---
  driver1.voltage_power_supply = 48;
  driver1.dead_zone          = 300e-9; // 300 ns
  driver1.pwm_frequency      = 15000;
  driver1.init();
  motor1.linkDriver(&driver1);
  motor1.controller    = MotionControlType::velocity_openloop;
  motor1.voltage_limit = 8;    // will be overwritten each cycle
  motor1.init();

  // --- Driver & Motor 2 init ---
  driver2.voltage_power_supply = 48;
  driver2.dead_zone          = 300e-9;
  driver2.pwm_frequency      = 15000;
  driver2.init();
  motor2.linkDriver(&driver2);
  motor2.controller    = MotionControlType::velocity_openloop;
  motor2.voltage_limit = 8;
  motor2.init();

  // seed timers
  t_prev = micros();
  last_command_time = micros();
}

void loop() {
  unsigned long now = micros();

  // Check for command timeout (1 second)           /////////////////////////////////////////////////
  //if (now - last_command_time > command_timeout) {
  //  target_voltage1 = 0;
  //  target_voltage2 = 0;
  //}

  // --- 10 kHz control update ---
  if (now - t_prev >= update_interval) {
    t_prev += update_interval;

    // update timing
    motor1.loopFOC();
    motor2.loopFOC();

    // Motor 1: map input voltage → velocity & voltage_limit
    {
      float Vin1 = fabs(target_voltage1);
      float v1   = 2.5 * Vin1;  // velocity mapping
      float y1;
      if (v1 <= 5) {
        y1 = 1.0 * v1;
      } else if (v1 <= 10) {
        y1 = 0.75 * v1;
      } else {
        y1 = 0.5 * v1;
      }
      motor1.target        = -v1;
      motor1.voltage_limit = y1;
      motor1.move();
    }

    // Motor 2: same mapping
    {
      float Vin2 = fabs(target_voltage2);
      float v2   = 2.5 * Vin2;
      float y2;
      if (v2 <= 5) {
        y2 = 1.0 * v2;
      } else if (v2 <= 10) {
        y2 = 0.75 * v2;
      } else {
        y2 = 0.5 * v2;
      }
      motor2.target        = -v2;
      motor2.voltage_limit = y2;
      motor2.move();
    }
  }

  // --- Serial command processing ---
  static String line = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      int id;
      float V;
      if (sscanf(line.c_str(), "%d %f", &id, &V) == 2) {
        switch (id) {
          case 1:
            target_voltage1 = V;
            Serial.printf("Motor 1 input voltage → %.2f V\n", V);
            break;
          case 2:
            target_voltage2 = V;
            Serial.printf("Motor 2 input voltage → %.2f V\n", V);
            break;
          case 3:
            target_voltage1 = V;
            target_voltage2 = V;
            Serial.printf("Both motors input voltage → %.2f V\n", V);
            break;
          default:
            Serial.println("Invalid motor ID (use 1, 2, or 3)");
        }
        last_command_time = now;  // reset timeout clock on valid command
      } else {
        Serial.println("Bad format. Use: <motor_id> <voltage>");
      }
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }
}
