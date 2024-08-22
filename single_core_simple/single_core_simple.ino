/* WARNING: The FastAccelStepper library is BROKEN with version 3.0 of the esp32 core. Open up the board manager and revert to version 2.0.17
 * 
 */

#include <TMCStepper.h>

/* STEP PIN SETUP:
*  Change these pins on the ESP32
*/
#define DIR_PIN 5
#define STEP_PIN 10
#define ENABLE_PIN 9
#define RX_PIN 19  // Set your RX pin on the ESP32
#define TX_PIN 22  // Set your TX pin on the ESP32
#define STALLGUARD_PIN 23
#define INDE_PIN 2
#define R_SENSE 0.12f     // R_SENSE for current calc.
#define DRIVER_ADDRESS 0  // TMC2209 Driver address according to MS1 and MS2

#define MOVE_VELOCITY 5000  // Sets the speed the motor will run

//Change these values to get different results
int32_t move_to_step = 3200;  //Change this value to set the position to move to (Negative will reverse)

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
// note: no update on stopMove()
//
// Returns 0 on success, or -1 on invalid value (<=0)
int32_t set_accel = 3200 * 100;  //Fast acceleration
int32_t set_current = 300;

// IF StallGuard does not work, it's because these two values are not set correctly or your pins are not correct.
int32_t set_stall = 80;    //Do not set the value too high or the TMC will not detect it. Start low and work your way up.
int32_t set_tcools = 285;  // Set slightly higher than the max TSTEP value you see
uint16_t motor_microsteps = 16;
int32_t tmc_steps = 0;

bool stalled_motor = false;
bool motor_moving = false;
bool isRunning = false;
bool isClosing = false;

// We communicate with the TMC2209 over UART
// But the Arduino UNO only have one Serial port which is connected to the Serial Monitor
// We can use software serial on the UNO, and hardware serial on the ESP32 or Mega 2560

TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);

// Interrupt
// This interrupt will fire when a HIGH rising signal is detected on the DIAG pin. This indicates a stall.

void IRAM_ATTR stalled_position() {
  stalled_motor = true;
}

// ## Track Steps
// When using the pulse generator, the TMC2209 will tell us each time a step has been taken by pulsing the INDEX pin. We can create a tracker to add or subract steps from the tracker
// The isClosing variable keeps track of which direction the motor is spinning, this way we know whether eto add or subract from the position.
void IRAM_ATTR index_interrupt() {
  // Track the position of the motor here

  if (isClosing == true) {
    tmc_steps++;
  } else {
    tmc_steps--;
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);  // ESP32 can use any pins to Serial

  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STALLGUARD_PIN, INPUT);
  pinMode(INDEX_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(STALLGUARD_PIN), stalled_position, RISING);
  attachInterrupt(digitalPinToInterrupt(INDEX_PIN), index_interrupt, RISING);

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
}


void loop() {
  // Add your Wi-Fi code here if required. The motor control will be done in the other task and core.

  if (isRunning == true) {

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
  }

  // The built in pulse generator cannot accelerate. To accelerate, create a for loop that starts at 0 velocity, and increases it's speed until the MOVE_VELOCITY speed is reached

  for (int i = 0; i < MOVE_VELOCITY; i = i + 100) {
    driver.VACTUAL(i);
  }

  driver.VACTUAL(MOVE_VELOCITY);  // We tell the motor to move to a spcific position.
  isRunning = true;

  if (isRunning() == true) {
    if (stalled_motor == true) {
      driver.VACTUAL(0);
      Serial.println("Stalled");
      stalled_motor = false;  // We'll set the stall flag to false, in case it was triggered easlier.
      delay(2000);            // Delay for testing
      break;                  // Break from the while loop
    }
    delay(1);  // We need to kill some time while the motor moves
  }
}