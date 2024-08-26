// ##### Finite State Machine Demo ######
// This is an example of a finite state machine for the TMC2209
// The states automatically change from BOOTUP -> HOME -> IDLE -> RUN_MAX -> RUN_ZERO
// Replace the content insdie the states, or add new states if you need. Add a button rather than delays. The sky's the limit!

// ## WARNING ##
// The FastAccelStepper library is BROKEN with version 3.0 of the esp32 core.
// Open up the board manager and revert to version 2.0.17

// ## WARNING ##
// ESP32-S3 may require fLash changes
// Go to Tools-> Flash Mode -> OPI mode
// Go to Tools-> PSRAM -> OPI mode

// Please always refer to the TMC2209 datasheet found here: https://www.analog.com/media/en/technical-documentation/data-sheets/tmc2209_datasheet_rev1.09.pdf

// ## TMCStepper Library
// For documentation go here: https://teemuatlut.github.io/TMCStepper/class_t_m_c2209_stepper.html
#include <TMCStepper.h>

// ## FastAccelStepper Library
// Documentation can be found in the header file here: https://github.com/gin66/FastAccelStepper/blob/master/src/FastAccelStepper.h
#include <FastAccelStepper.h>

// ## STEP PIN SETUP:
// Change these pins on the ESP32
#define DIR_PIN 5
#define STEP_PIN 15        
#define ENABLE_PIN 16       
#define RX_PIN 19          
#define TX_PIN 20          
#define STALLGUARD_PIN 21

enum states {
  BOOTUP,
  HOME,
  IDLE,
  STALL,
  RUN_ZERO,
  RUN_MAX
};

states prior_state, state;

// ## Position
// Change these values to set the maximum step position.
int32_t maxStep = 3200 * 10;  //Change this value to set the position to move to (Negative will reverse)

// ## Speed
// Sets the speed in microsteps per second.
// If for example the the motor_microsteps is set to 16 and your stepper motor has 200 full steps per revolution (Most common type. The motor angle will be 1.8 degrees on the datasheet)
// It means 200 x 16 = 3200 steps per revolution. To set the speed to rotate one revolution per seconds, we would set the value below to 3200.
int32_t set_velocity = 3200;

// ## Acceleration
//  setAcceleration() expects as parameter the change of speed
//  as step/s².
//  If for example the speed should ramp up from 0 to 10000 steps/s within
//  10s, then the acceleration is 10000 steps/s / 10s = 1000 steps/s²
//
// New value will be used after call to
// move/moveTo/runForward/runBackward/applySpeedAcceleration/moveByAcceleration
//
// Returns 0 on success, or -1 on invalid value (<=0)
int32_t set_accel = 3200 * 100;  // We're using a fast acceleration for this example.

// ## Current
// Always keep you current as low as possible for your application
int32_t set_current = 300;

// ## SGTHRS
// IF StallGuard does not work, it's because these two values are not set correctly or your pins are not correct.
int32_t set_stall = 80;  //Do not set the value too high or the TMC will not detect it. Start low and work your way up.

// ## TCOOLTHRS
int32_t set_tcools = 285;  //Set 1.2 times higher than the max TSTEP value you see

// ## Microsteps
uint16_t motor_microsteps = 16;

// Stall variables. Do not change.
bool stalled_motor = false;
bool motor_moving = false;

// We communicate with the TMC2209 over UART
// But the Arduino UNO only have one Serial port which is connected to the Serial Monitor
// We can use software serial on the UNO, and hardware serial on the ESP32 or Mega 2560

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
TMC2209Stepper driver(&Serial1, 0.12f, 0);

// ## Interrupt
// This interrupt will fire when a HIGH rising signal is detected on the DIAG pin. This indicates a stall.
void IRAM_ATTR stalled_position() {
  stalled_motor = true;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // ESP32 can use any pins to Serial

  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STALLGUARD_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(STALLGUARD_PIN), stalled_position, RISING);

  driver.begin();                       // Start all the UART communications functions behind the scenes
  driver.toff(4);                       //For operation with StealthChop, this parameter is not used, but it is required to enable the motor. In case of operation with StealthChop only, any setting is OK
  driver.blank_time(24);                //Recommended blank time select value
  driver.I_scale_analog(false);         // Disbaled to use the extrenal current sense resistors
  driver.internal_Rsense(false);        // Use the external Current Sense Resistors. Do not use the internal resistor as it can't handle high current.
  driver.mstep_reg_select(true);        //Microstep resolution selected by MSTEP register and NOT from the legacy pins.
  driver.microsteps(motor_microsteps);  //Set the number of microsteps. Due to the "MicroPlyer" feature, all steps get converterd to 256 microsteps automatically. However, setting a higher step count allows you to more accurately more the motor exactly where you want.
  driver.TPWMTHRS(0);                   //DisableStealthChop PWM mode/ Page 25 of datasheet
  driver.semin(0);                      // Turn off smart current control, known as CoolStep. It's a neat feature but is more complex and messes with StallGuard.
  driver.shaft(true);                   // Set the shaft direction. Only use this command one time during setup to change the direction of your motor. Do not use this in your code to change directions.
  driver.en_spreadCycle(false);         // Disable SpreadCycle. We want StealthChop becuase it works with StallGuard.
  driver.pdn_disable(true);             // Enable UART control
  driver.VACTUAL(0);                    // Enable UART control
  driver.rms_current(set_current);
  driver.SGTHRS(set_stall);
  driver.TCOOLTHRS(set_tcools);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(ENABLE_PIN);
  stepper->setAutoEnable(true);  // This sets the enable pin and turns the TMC2209 on/off atomatically
  stepper->setSpeedInHz(set_velocity);
  stepper->setAcceleration(set_accel);

  // Now set up tasks to run independently.
  xTaskCreatePinnedToCore(
    MotorTask  //Motor Task
    ,
    "MotorTask"  // A name just for humans
    ,
    1024 * 4  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,
    NULL, 3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,
    NULL, 0);

  prior_state = BOOTUP;
  state = HOME;

  Serial.println("Setup Complete");
}

void loop() {
  // Add your Wi-Fi code here if required. The motor control will be done in the other task and core.

  while (stepper->isRunning() == true) {

    // ## STEP 1: Uncomment this line below and obtain the value from the serial monitor.
    // Multiply this obtained value by 1.2 and set the variable "set_tcools" to this value on line 29
    // Serial.println(driver.TSTEP());
    // Now comment the line above

    // ## STEP 2: Uncomment the line below and watch the value change in the serial monitor. This is the SG_RESULT value
    // Serial.println(driver.SG_RESULT());
    // If the value is averaging 260, then a SGTHRS value of 260/2 (130) will trigger the stall.
    // We want a SGTHRS value that's much smaller as to not trigger a stall unintentioally. A value of 80 is ok becuase it means the SG_RESULT must drop to 160 (80x2) before a stall is triggered.
    // Update the "set_stall" variable on line 28 with your desired value.

    // ## Monitor the position if desired
    //Serial.println(stepper->getCurrentPosition());

    // The motor should now stop automatically when a stall occurs, delay for 2 seconds, then spin in the opposite direction.

    delay(50);
  }
}

// ## HOME STATE ##
// Create a homing sequence here.
// In this example, the motor will run backwards for 2 seconds and set the position to zero.
// Use this to locate a limit switch, or maybe use stallGuard to detect a stall
void home() {
  if (state != prior_state) {  // If we are entering the state, do some actions
    Serial.println("HOME");
    stepper->setCurrentPosition(0);  // Reset the home position to 0 before starting.
    prior_state = state;
  }

  // ****** Add homing actions here ********
  stepper->runBackward();          // Run the motor backwards to look for a hypothetical limit switch.
  delay(5000);                     // Let the motor run for 2 seconds, maybe until it hits a limit switch or stallGuard
  stepper->setCurrentPosition(0);  // We'll set the position to 0

  state = IDLE;  // Toggle the STATE once homing is complete

  if (state != prior_state) {  // If we are leaving the state, do some one-time actions
  }
}

// ## IDLE STATE ##
void idle() {
  if (state != prior_state) {  // If we are entering the state, do some actions
    Serial.println("IDLE");
    prior_state = state;
  }

  // ****** Add idle actions here ********
  delay(2000);  // Delay 3 seconds for fun

  state = RUN_MAX;  // Toggle the STATE once idle is complete

  if (state != prior_state) {  // If we are leaving the state, do some actions
  }
}


// ## STALL STATE ##
void stall() {
  if (state != prior_state) {  // If we are entering the state, do some actions
    stepper->forceStop();
    Serial.println("YOU HAVE STALLED!");
    stalled_motor = false;
    prior_state = state;
  }

  // ****** Add stall actions here ********
  delay(100);  // Delay for 2 seconds

  state = IDLE;  // Set to IDLE state.

  if (state != prior_state) {  // If we are leaving the state, do some actions
    // Exit actions go here. Turn off lights, turn off alarm, etc.
  }
}

// ## RUN MAX STATE ##
void run_max() {
  if (state != prior_state) {  // If we are entering the state, do some actions
    // One time state actions go here
    Serial.println("RUN_MAX");
    stepper->moveTo(maxStep);
    prior_state = state;
  }

  // ****** Add run actions here ********
  if (stalled_motor == true) {
    state = STALL;
  }

  if (stepper->isRunning() == false) {
    state = RUN_ZERO;  // Toggle the STATE once homing is complete
  }

  delay(10);  // Delay to allow other processes while we wait. Keeps loop from crashing

  if (state != prior_state) {  // If we are leaving the state, do some actions
  }
}

// ## RUN ZERO STATE ##
void run_zero() {
  if (state != prior_state) {  // If we are entering the state, do some actions
    // One time state actions go here
    Serial.println("RUN_ZERO");
    stepper->moveTo(0);
    prior_state = state;
  }

  // ****** Add run actions here ********
  if (stalled_motor == true) {
    state = STALL;
  }

  if (stepper->isRunning() == false) {
    state = IDLE;  // Toggle the STATE once homing is complete
  }

  delay(10);  // Delay to allow other processes while we wait. Keeps loop from crashing

  if (state != prior_state) {  // If we are leaving the state, do some actions
  }
}



void MotorTask(void *pvParameters) {

  while (true) {

    switch (state) {
      case BOOTUP:
        setup();
        break;
      case HOME:
        home();
        break;
      case IDLE:
        idle();
        break;
      case STALL:
        stall();
        break;
      case RUN_ZERO:
        run_zero();
        break;
      case RUN_MAX:
        run_max();
        break;
    }
  }
}