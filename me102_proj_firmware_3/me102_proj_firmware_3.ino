// import libraries
#include <Encoder.h>
#include <HCSR04.h> // https://github.com/Martinsos/arduino-lib-hc-sr04
#include "TimerInterrupt.h"
// Specify GPIO Mapping
// led
#define led_green A5
#define led_yellow A4
#define led_red A3
// hc_sr04
#define hc_echo A1
#define hc_trig A0
// button
#define button 7
// force_sensor
#define fsensor A2 // A3
// motors
#define mdriver_1_dir 13
#define mdriver_1_pwm 11
#define mdriver_1_enc1 3
#define mdriver_1_enc2 10
#define mdriver_2_dir 8
#define mdriver_2_pwm 9
#define mdriver_2_enc1 2
#define mdriver_2_enc2 6
#define mdriver_3_dir 4
#define mdriver_3_pwm 5
// constants
#define STATE_IDLE 0
#define STATE_COLLECTING 1
#define STATE_FULL 2
#define STATE_EMPTYING 3
#define STATE_ERROR 4
#define MIN_DISTANCE 5          // cm
#define FORCE_SENSOR_THRESH 620 // 2V / 3.3V * 1023
#define ERROR_MAX 30
// PWM properties
#define MAX_PWM_VOLTAGE 255
#define NOM_PWM_VOLTAGE 150
#define OMEGA_DES_DRIVE 53
// Timers
#define USE_TIMER_1 true
#define USE_TIMER_2 true
#define TIMER1_INTERVAL_MS 50L
#define TIMER2_INTERVAL_MS 100L
#define SENSOR_INTERVAL_MS 20
#define LOGIC_INTERVAL_US 1000
#define LED_INTERVAL_MS 50
#define MOTOR_ON_TIME_MS 5000
#define BUTTON_DEBOUNCE_TIMER 250
#define DRIVE_FEEDBACK_TIMER 100

Encoder encDriveLeft(mdriver_1_enc1, mdriver_1_enc2);
Encoder encDriveRight(mdriver_2_enc1, mdriver_2_enc2);
UltraSonicDistanceSensor distanceSensor(hc_trig, hc_echo);

// global variables
int global_state;
volatile bool buttonIsPressed;
volatile float force_sensor_reading;
volatile float force_sensor_reading_acc;
volatile float hc_distance_acc; // cm
volatile float hc_distance;     // cm
volatile bool drive_counter;
bool driving;
bool emptying;
volatile bool led_on;
long positionLeft;
long positionRight;
bool led_red_flashing;
bool led_yellow_flashing;
long drive_time;
long led_time;
long loop_time;
volatile long current_time_US;
volatile long current_time_MS;
bool button_timer_active;
long button_timer;
long sensor_time;
long drive_time2;
int error_sum_right;
int error_sum_left;

// Initialization

void setup()
{
  // set global variable initial value
  global_state = 0;
  buttonIsPressed = false;
  force_sensor_reading = 0;
  force_sensor_reading_acc = 0;
  drive_counter = false;
  hc_distance = 0;
  hc_distance_acc = 0;
  driving = false;
  emptying = false;
  led_on = false;
  led_red_flashing = false;
  led_yellow_flashing = false;
  positionLeft = 0;
  positionRight = 0;
  drive_time = 0;
  led_time = 0;
  loop_time = 0;
  current_time_US = 0;
  current_time_MS = 0;
  button_timer_active = false;
  button_timer = 0;
  sensor_time = 0;
  drive_time2 = 0;
  error_sum_right = 0;
  error_sum_left = 0;

  // assign pins
  pinMode(button, INPUT);
  pinMode(led_green, OUTPUT);
  pinMode(led_yellow, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(fsensor, INPUT);
  pinMode(mdriver_1_dir, OUTPUT);
  pinMode(mdriver_1_pwm, OUTPUT);
  pinMode(mdriver_2_dir, OUTPUT);
  pinMode(mdriver_2_pwm, OUTPUT);
  pinMode(mdriver_3_dir, OUTPUT);
  pinMode(mdriver_3_pwm, OUTPUT);
  pinMode(mdriver_1_enc1, INPUT);
  pinMode(mdriver_1_enc2, INPUT);
  pinMode(mdriver_2_enc1, INPUT);
  pinMode(mdriver_2_enc2, INPUT);
  pinMode(button, INPUT);
  setYellowLED();
  encDriveLeft.write(0);
  encDriveRight.write(0);

  Serial.begin(115200);
}

void loop()
{
  current_time_US = micros();
  current_time_MS = millis();
  if (current_time_US - loop_time >= LOGIC_INTERVAL_US)
  {
    if (current_time_MS - led_time >= LED_INTERVAL_MS)
    {
      if (led_yellow_flashing)
      {
        if (led_on)
        {
          digitalWrite(led_yellow, LOW);
          led_on = false;
        }
        else
        {
          digitalWrite(led_yellow, HIGH);
          led_on = true;
        }
      }
      else if (led_red_flashing)
      {
        if (led_on)
        {
          digitalWrite(led_red, LOW);
          led_on = false;
        }
        else
        {
          digitalWrite(led_red, HIGH);
          led_on = true;
        }
      }
      led_time = current_time_MS;
    }
    if (current_time_MS - sensor_time >= SENSOR_INTERVAL_MS)
    {
      refresh_sensors();
      switch (global_state)
      {
      case STATE_IDLE:
        Serial.println("STATE_IDLE");
        // LED should be yellow, all motors are turned off

        if (check_weight())
        {
          global_state = STATE_FULL;
          routine3();
        }

        if (buttonPressEvent())
        {
          global_state = STATE_COLLECTING;
          routine1();
        }
        Serial.print("HC Distance: ");
        Serial.println(hc_distance);
        Serial.print("Force Sensor: ");
        Serial.println(force_sensor_reading);
        break;

      case STATE_COLLECTING:
        Serial.println("STATE_COLLECTING");
        // LED should be green, linkage motor is off, drive motors on (preset routine)
        if (check_distance())
        {
          global_state = STATE_ERROR;
          routine2();
        }

        if (buttonPressEvent())
        {
          global_state = STATE_IDLE;
          routine4();
        }

        if (check_weight())
        {
          global_state = STATE_FULL;
          routine3();
        }

        drive_routine();

        break;

      case STATE_FULL:
        Serial.println("STATE_FULL");
        // LED should be red, all motors are turned off

        if (buttonPressEvent())
        {
          global_state = STATE_EMPTYING;
          routine5();
        }
        break;

      case STATE_EMPTYING:
        Serial.println("STATE_EMPTYING");
        // LED should be blinking yellow, linkage motor is on, drive motors off
        if (check_distance())
        {
          global_state = STATE_ERROR;
          routine2();
        }

        empty_routine();

        break;

      case STATE_ERROR:
        Serial.println("STATE_ERROR");
        // LED should be blinking red, all motors off
        break;

      default:
        Serial.println("SM_ERROR");
        global_state = STATE_ERROR;
        routine2();
        break;
      }
      sensor_time = current_time_MS;
    }
    else
    {
      poll_sensors();
    }
    loop_time = current_time_US;
  }
}

// Event Checkers
bool buttonPressEvent()
{
  if (button_timer_active)
  {
    if (current_time_MS - button_timer >= BUTTON_DEBOUNCE_TIMER)
    {
      buttonIsPressed = false;
      button_timer_active = false;
      return true;
    }
    return false;
  }
  if (buttonIsPressed)
  {
    button_timer = current_time_MS;
    button_timer_active = true;
  }
  return false;
}

void poll_sensors()
{
  force_sensor_reading_acc = force_sensor_reading_acc + analogRead(fsensor);
  hc_distance_acc = hc_distance_acc + distanceSensor.measureDistanceCm();
  int buttonState = digitalRead(button);
  if (buttonState == HIGH) {
    buttonIsPressed = true;
  }
}

void refresh_sensors()
{

  Serial.println("Refreshing sensors");
  hc_distance = hc_distance_acc / SENSOR_INTERVAL_MS;
  hc_distance_acc = 0;
  force_sensor_reading = force_sensor_reading_acc / SENSOR_INTERVAL_MS;
  force_sensor_reading_acc = 0;
  // hc_distance = distanceSensor.measureDistanceCm();
  // force_sensor_reading = analogRead(fsensor);
}

bool check_weight()
{
  if (force_sensor_reading < FORCE_SENSOR_THRESH)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool check_distance()
{
  if (hc_distance < MIN_DISTANCE && hc_distance >= 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}

// Event Service Response

// Routines
void routine1() // start collecting
{
  Serial.println("Running routine 1");
  setGreenLED();
  stopLinkageMotor();
  startDriveMotors();
}

void routine2() // error
{
  Serial.println("Running routine 2");
  setRedLEDflashing();
  stopDriveMotors();
  stopLinkageMotor();
}

void routine3() // full
{
  Serial.println("Running routine 3");
  setRedLED();
  stopDriveMotors();
  stopLinkageMotor();
}

void routine4() // idle
{
  Serial.println("Running routine 4");
  setYellowLED();
  stopDriveMotors();
  stopLinkageMotor();
}

void routine5() // emptying
{
  Serial.println("Running routine 5");
  setYellowLEDflashing();
  stopDriveMotors();
  startLinkageMotor();
}

// Subroutines
void drive_routine()
{
  if (driving)
  {
    if (current_time_MS - drive_time >= MOTOR_ON_TIME_MS)
    {
      global_state = STATE_IDLE;
      routine4();
    }

    if (current_time_MS - drive_time2 >= DRIVE_FEEDBACK_TIMER)
    {
      int omega_des_local = OMEGA_DES_DRIVE / 10;
      int real_count_right = encDriveRight.read();
      int real_count_left = encDriveLeft.read();

      int Kp = 20;
      int Ki = 60;
      int error_right = omega_des_local - real_count_right;
      int error_left = omega_des_local - real_count_left;
      int Dr = Kp * error_right + Ki * error_sum_right / 10;
      int Dl = Kp * error_left + Ki * error_sum_left / 10;
      error_sum_right += error_right;
      error_sum_left += error_left;
      //Ensure that the error doesn't get too high
      if (error_sum_right > ERROR_MAX)
      {
        error_sum_right = ERROR_MAX;
      }
      else if (error_sum_right < -ERROR_MAX)
      {
        error_sum_right = -ERROR_MAX;
      }
      if (error_sum_left > ERROR_MAX)
      {
        error_sum_left = ERROR_MAX;
      }
      else if (error_sum_left < -ERROR_MAX)
      {
        error_sum_left = -ERROR_MAX;
      }

      //Ensure that you don't go past the maximum possible command
      if (Dr > MAX_PWM_VOLTAGE)
      {
        Dr = MAX_PWM_VOLTAGE;
      }
      else if (Dr < -MAX_PWM_VOLTAGE)
      {
        Dr = -MAX_PWM_VOLTAGE;
      }
      if (Dl > MAX_PWM_VOLTAGE)
      {
        Dl = MAX_PWM_VOLTAGE;
      }
      else if (Dl < -MAX_PWM_VOLTAGE)
      {
        Dl = -MAX_PWM_VOLTAGE;
      }

      analogWrite(mdriver_1_pwm, Dr);
      analogWrite(mdriver_2_pwm, Dl);

      drive_time2 = current_time_MS;
      encDriveLeft.write(0);
      encDriveRight.write(0);
    }
  }
}

void empty_routine()
{
  if (emptying)
  {
    if (current_time_MS - drive_time >= MOTOR_ON_TIME_MS)
    {
      global_state = STATE_IDLE;
      routine4();
    }
  }
}

void setGreenLED()
{
  digitalWrite(led_green, HIGH);
  digitalWrite(led_yellow, LOW);
  digitalWrite(led_red, LOW);
  led_yellow_flashing = false;
  led_red_flashing = false;
}

void setYellowLED()
{
  digitalWrite(led_green, LOW);
  digitalWrite(led_yellow, HIGH);
  digitalWrite(led_red, LOW);
  led_yellow_flashing = false;
  led_red_flashing = false;
}

void setRedLED()
{
  digitalWrite(led_green, LOW);
  digitalWrite(led_yellow, LOW);
  digitalWrite(led_red, HIGH);
  led_yellow_flashing = false;
  led_red_flashing = false;
}

void setYellowLEDflashing()
{
  digitalWrite(led_green, LOW);
  digitalWrite(led_yellow, HIGH);
  digitalWrite(led_red, LOW);
  led_yellow_flashing = true;
  led_red_flashing = false;
  led_on = true;
  led_time = current_time_MS;
}

void setRedLEDflashing()
{
  digitalWrite(led_green, LOW);
  digitalWrite(led_yellow, LOW);
  digitalWrite(led_red, HIGH);
  led_yellow_flashing = false;
  led_red_flashing = true;
  led_on = true;
  led_time = current_time_MS;
}

void startDriveMotors()
{
  // left motor
  analogWrite(mdriver_1_dir, LOW);
  analogWrite(mdriver_1_pwm, 0);
  // right motor
  analogWrite(mdriver_2_dir, LOW);
  analogWrite(mdriver_2_pwm, 0);

  driving = true;
  drive_time = current_time_MS;
  drive_time2 = current_time_MS;
  encDriveLeft.write(0);
  encDriveRight.write(0);
  error_sum_right = 0;
  error_sum_left = 0;
}

void stopDriveMotors()
{
  // left motor
  analogWrite(mdriver_1_dir, LOW);
  analogWrite(mdriver_1_pwm, 0);
  // right motor
  analogWrite(mdriver_2_dir, LOW);
  analogWrite(mdriver_2_pwm, 0);

  driving = false;
}

void startLinkageMotor()
{
  analogWrite(mdriver_3_dir, LOW);
  analogWrite(mdriver_3_pwm, NOM_PWM_VOLTAGE);

  emptying = true;
  drive_time = current_time_MS;
}

void stopLinkageMotor()
{
  analogWrite(mdriver_3_pwm, 0);

  emptying = false;
}
