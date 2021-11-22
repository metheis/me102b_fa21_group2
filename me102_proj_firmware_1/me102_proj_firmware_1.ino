// import libraries
#include <ESP32Encoder.h>
#include <HCSR04.h> // https://github.com/Martinsos/arduino-lib-hc-sr04
// Specify GPIO Mapping
// led
#define led_green A0
#define led_yellow A1
#define led_red A2
// hc_sr04
#define hc_echo A3
#define hc_trig A4
// button
#define button A5
// force_sensor
#define fsensor A6
// motors
#define mdriver_1_in1 A8
#define mdriver_1_in2 A7
#define mdriver_2_in1 A12
#define mdriver_2_in2 A11
#define mdriver_2_in3 A10
#define mdriver_2_in4 A9
// constants
#define STATE_IDLE 0
#define STATE_COLLECTING 1
#define STATE_FULL 2
#define STATE_EMPTYING 3
#define STATE_ERROR 4
#define MIN_DISTANCE 5 // cm
#define FORCE_SENSOR_THRESH 30
// PWM properties
#define freq 5000
#define ledChannel_1 1
#define ledChannel_2 2
#define ledChannel_3 3
#define ledChannel_4 4
#define ledChannel_5 5
#define ledChannel_6 6
#define resolution 8
#define MAX_PWM_VOLTAGE 255
#define NOM_PWM_VOLTAGE 150
#define LINKAGE_PWM_VOLTAGE 100
#define PWM_RANGE 4096

ESP32Encoder encoder;
UltraSonicDistanceSensor distanceSensor(hc_trig, hc_echo);

// global variables
int global_state;
hw_timer_t *timer0;
portMUX_TYPE timerMux0;
hw_timer_t *timer1;
portMUX_TYPE timerMux1;
hw_timer_t *timer2;
portMUX_TYPE timerMux2;
volatile bool buttonIsPressed;
volatile float force_sensor_reading;
volatile bool sensor_counter; // check timer interrupt 1
volatile bool deltaT;         // check timer interrupt 2
volatile float hc_distance;   // cm
volatile bool drive_counter;
int drive_duty_cycle;
int linkage_duty_cycle;
bool driving;
bool emptying;
bool led_on;

// Initialization
void IRAM_ATTR onTime0()
{
  portENTER_CRITICAL_ISR(&timerMux0);
  sensor_counter = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void IRAM_ATTR onTime1()
{
  portENTER_CRITICAL_ISR(&timerMux1);
  // count = encoder.getCount();
  // encoder.clearCount();
  deltaT = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux1);
}

void IRAM_ATTR onTime2()
{
  portENTER_CRITICAL_ISR(&timerMux2);
  drive_counter = true; // the function to be called when timer interrupt is triggered
  portEXIT_CRITICAL_ISR(&timerMux2);
}

void IRAM_ATTR isr()
{ // the function to be called when interrupt is triggered
  buttonIsPressed = true;
}

void setup()
{
  // set global variable initial value
  global_state = 0;
  timer0 = NULL;
  timer1 = NULL;
  timer2 = NULL;
  timerMux0 = portMUX_INITIALIZER_UNLOCKED;
  timerMux1 = portMUX_INITIALIZER_UNLOCKED;
  timerMux2 = portMUX_INITIALIZER_UNLOCKED;
  buttonIsPressed = false;
  force_sensor_reading = 0;
  sensor_counter = false;
  drive_counter = false;
  deltaT = false;
  hc_distance = 0;
  drive_duty_cycle = NOM_PWM_VOLTAGE / MAX_PWM_VOLTAGE * PWM_RANGE;
  linkage_duty_cycle = LINKAGE_PWM_VOLTAGE / MAX_PWM_VOLTAGE * PWM_RANGE;
  driving = false;
  emptying = false;
  led_on = false;

  // assign pins
  pinMode(button, INPUT);
  pinMode(led_green, OUTPUT);
  pinMode(led_yellow, OUTPUT);
  pinMode(led_red, OUTPUT);
  pinMode(fsensor, INPUT);
  digitalWrite(led_green, LOW); // sets the initial state of LED as turned-off
  digitalWrite(led_yellow, LOW);
  digitalWrite(led_red, LOW);
  attachInterrupt(button, isr, RISING);

  Serial.begin(115200);

  // configure LED PWM functionalitites
  ledcSetup(ledChannel_1, freq, resolution);
  ledcSetup(ledChannel_2, freq, resolution);
  ledcSetup(ledChannel_3, freq, resolution);
  ledcSetup(ledChannel_4, freq, resolution);
  ledcSetup(ledChannel_5, freq, resolution);
  ledcSetup(ledChannel_6, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(mdriver_1_in1, ledChannel_1);
  ledcAttachPin(mdriver_1_in2, ledChannel_2);
  ledcAttachPin(mdriver_2_in1, ledChannel_3);
  ledcAttachPin(mdriver_2_in2, ledChannel_4);
  ledcAttachPin(mdriver_2_in3, ledChannel_5);
  ledcAttachPin(mdriver_2_in4, ledChannel_6);

  // initilize timer
  timer0 = timerBegin(0, 80, true);             // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer0, &onTime0, true); // edge (not level) triggered
  timerAlarmWrite(timer0, 100000, true);        // 100000 * 1 us = 100 ms, autoreload true

  timer1 = timerBegin(1, 80, true);             // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer1, &onTime1, true); // edge (not level) triggered
  timerAlarmWrite(timer1, 10000, true);         // 10000 * 1 us = 10 ms, autoreload true

  timer2 = timerBegin(1, 80, true);             // timer 1, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer2, &onTime2, true); // edge (not level) triggered
  timerAlarmWrite(timer2, 5000000, true);       // 5000000 * 1 us = 10 ms, autoreload true

  // enable timer
  timerAlarmEnable(timer0);
  timerAlarmEnable(timer1);
  timerAlarmEnable(timer2);
  timerStop(timer2);
}

void loop()
{
  switch (global_state)
  {
  case STATE_IDLE:
    Serial.println("STATE_IDLE");
    // LED should be yellow, all motors are turned off
    refresh_sensors();

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
    break;

  case STATE_COLLECTING:
    Serial.println("STATE_COLLECTING");
    // LED should be green, linkage motor is off, drive motors on (preset routine)
    refresh_sensors();

    if (hc_distance <= MIN_DISTANCE)
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
    refresh_sensors();

    if (hc_distance <= MIN_DISTANCE)
    {
      global_state = STATE_ERROR;
      routine2();
    }

    if (buttonPressEvent())
    {
      global_state = STATE_EMPTYING;
      routine5();
    }
    break;

  case STATE_EMPTYING:
    Serial.println("STATE_EMPTYING");
    // LED should be blinking yellow, linkage motor is on, drive motors off
    refresh_sensors();

    if (hc_distance <= MIN_DISTANCE)
    {
      global_state = STATE_ERROR;
      routine2();
    }
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
}

// Event Checkers
bool buttonPressEvent()
{
  if (buttonIsPressed == true)
  {
    buttonIsPressed = false;
    return true;
  }
  else
  {
    return false;
  }
}

void refresh_sensors()
{
  if (sensor_counter)
  {
    portENTER_CRITICAL(&timerMux0);
    sensor_counter = false;
    portEXIT_CRITICAL(&timerMux0);

    hc_distance = distanceSensor.measureDistanceCm();
    force_sensor_reading = analogRead(fsensor);
  }
}

bool check_weight()
{
  if (force_sensor_reading > FORCE_SENSOR_THRESH)
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
void routine1()
{
  setGreenLED();
  stopLinkageMotor();
  startDriveMotors();
}

void routine2()
{
  setRedLEDflashing();
  stopDriveMotors();
  stopLinkageMotor();
}

void routine3()
{
  setRedLED();
  stopDriveMotors();
  stopLinkageMotor();
}

void routine4()
{
  setYellowLED();
  stopDriveMotors();
  stopLinkageMotor();
}

void routine5()
{
  setYellowLEDflashing();
  stopDriveMotors();
  startLinkageMotor();
}

// Subroutines
void drive_routine()
{
  if (driving)
  {
    if (drive_counter)
    {
      routine4();
    }
  }
}

void empty_routine()
{
  if (emptying)
  {
    if (drive_counter)
    {
      routine4();
    }
  }
}

void setGreenLED()
{
  digitalWrite(led_green, HIGH);
  digitalWrite(led_yellow, LOW);
  digitalWrite(led_red, LOW);
}

void setYellowLED()
{
  digitalWrite(led_green, LOW);
  digitalWrite(led_yellow, HIGH);
  digitalWrite(led_red, LOW);
}

void setRedLED()
{
  digitalWrite(led_green, LOW);
  digitalWrite(led_yellow, LOW);
  digitalWrite(led_red, HIGH);
}

void setYellowLEDflashing()
{
  digitalWrite(led_green, LOW);
  digitalWrite(led_yellow, HIGH);
  digitalWrite(led_red, LOW);
}

void setRedLEDflashing()
{
  digitalWrite(led_green, LOW);
  digitalWrite(led_yellow, LOW);
  digitalWrite(led_red, HIGH);
}

void startDriveMotors()
{
  // left motor
  ledcWrite(ledChannel_1, LOW);
  ledcWrite(ledChannel_2, drive_duty_cycle); // high = forward
  // right motor
  ledcWrite(ledChannel_3, drive_duty_cycle); // high = forward
  ledcWrite(ledChannel_4, LOW);

  timerRestart(timer2);
  driving = true;
}

void stopDriveMotors()
{
  // left motor
  ledcWrite(ledChannel_1, LOW);
  ledcWrite(ledChannel_2, LOW);
  // right motor
  ledcWrite(ledChannel_3, LOW);
  ledcWrite(ledChannel_4, LOW);

  portENTER_CRITICAL(&timerMux2);
  drive_counter = false;
  portEXIT_CRITICAL(&timerMux2);
  driving = false;
  timerStop(timer2);
  timerWrite(timer2, 0);
}

void startLinkageMotor()
{
  ledcWrite(ledChannel_5, LOW);
  ledcWrite(ledChannel_6, linkage_duty_cycle); // high = up

  timerRestart(timer2);
  emptying = true;
}

void stopLinkageMotor()
{
  ledcWrite(ledChannel_5, LOW);
  ledcWrite(ledChannel_6, LOW);

  portENTER_CRITICAL(&timerMux2);
  drive_counter = false;
  portEXIT_CRITICAL(&timerMux2);
  driving = false;
  timerStop(timer2);
  timerWrite(timer2, 0);
}
