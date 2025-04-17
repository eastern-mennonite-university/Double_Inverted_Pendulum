#include <PID_v2.h>
#include <TimerOne.h>

// Stop Button
int stopButtonPin = 52;
int buttonState = 0;

// Encoder pins
const uint8_t ENCODER_A = 2;  // Interrupt pin for pendulum encoder
const uint8_t ENCODER_B = 3;  // Digital pin for pendulum encoder
const uint8_t CART_ENCODER_A = 18;  // Interrupt pin for cart position
const uint8_t CART_ENCODER_B = 19;  // Digital pin for cart position

// Stepper motor control pins
const uint8_t STEP_PIN = 10;
const uint8_t DIR_PIN = 11;
const uint8_t MF_PIN = 12;

// Time tracking
unsigned long startTime = millis();
unsigned long timeElapsed;
bool failstate = false;

// Encoder position tracking
volatile long rawPendulumPosition = 0;
volatile long rawCartPosition = 0;
double cartPosition = 0;
double maxOffset = 12;

// Pendulum Control PID
double angle = 0, setpoint = 0, pendulumOutput = 0, toperror = 0;
const double Kp = 4, Ki = 66, Kd = 0;
PID pendulumPID(&toperror, &pendulumOutput, &setpoint, Kp, Ki, Kd, DIRECT);

// Cart Position Control PID
double cartSetpoint = 0, cartOutput = 0;
//const double Kp_cart = 2.0, Ki_cart = 20, Kd_cart = 5;  // Adjust based on tuning
const double Kp_cart = 1.6, Ki_cart = 1.0, Kd_cart = 2.3;
PID cartPID(&cartPosition, &cartOutput, &cartSetpoint, Kp_cart, Ki_cart, Kd_cart, DIRECT);

// Stepper parameters
const int MAX_STEP_SPEED = 8000;  // Max step rate

// Stepper movement setup
volatile long stepsRemaining = 0;
volatile bool stepState = false;
volatile bool isStepping = false;


// Helper to calculate total movement time
unsigned long getStepDurationMicros(long steps, long intervalMicros) {
  return abs(steps) * 2L * intervalMicros; // 2 toggles per step
}

//Changing cart position
int state = 1; //starts at zero
int positions[4] = {-200, 0, 200, 0};



void setup() {
    Serial.begin(115200);
    digitalWrite(MF_PIN, HIGH); // Disables motor

    // Stop Button setup
    pinMode(stopButtonPin, INPUT_PULLUP);

    // Pendulum encoder setup
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoderISR, CHANGE);

    // Cart pendulum encoder setup
    pinMode(CART_ENCODER_A, INPUT_PULLUP);
    pinMode(CART_ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CART_ENCODER_A), cartEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(CART_ENCODER_B), cartEncoderISR, CHANGE);
    
    // Stepper setup
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(MF_PIN, OUTPUT);

    // PID setup
    pendulumPID.SetMode(AUTOMATIC);
    pendulumPID.SetOutputLimits(-8000, 8000); // Range of PID output for pendulum
    pendulumPID.SetSampleTime(10);

    cartPID.SetMode(AUTOMATIC);
    cartPID.SetOutputLimits(-1000, 1000);  // Range of PID output for cart
    cartPID.SetSampleTime(10);

    Timer1.initialize(100);  // Dummy value, will be updated when stepping starts
    Timer1.attachInterrupt(stepperISR);  // ISR that toggles STEP pin
    Timer1.stop();  // Don't run until commanded

    //initialization();
    digitalWrite(MF_PIN, LOW); // Enables Motor

    delay(1000); // Initial delay before running
    swingup();


}

void loop() {

  while (failstate == false) {

      buttonState = digitalRead(stopButtonPin); // Reads EM stop button
      balance();
      serialprint();
      delay(1); // Removes a some oscillation

     if(abs(angle) > 15 || abs(cartPosition) > 325 || buttonState == 1) { // Checks all three failstates, max angle, max position and EM button
        failstate = true;
        digitalWrite(MF_PIN, HIGH);
        delay(1000);
        swingup(); // Runs swingup to reset pendulum
      }
    
  }

}

void balance() {

    // Compute angle from pendulum encoder position
    angle = rawPendulumPosition * .045;  // Second value is Degrees/Pulse = 360 / 8000 (8000 = pendulum encoder resolution)
    toperror = sin(angle*3.14159/180)*320; // Calculates strait line distance error using trig

    // Compute cart distance from cart encoder position
    cartPosition = rawCartPosition * .03; // Second value is mm/pulse = 120 / 4000 (120 = circumference of motor pulley, 4000 = motor encoder resolution)

    // Run PID controllers
    cartPID.Compute();

    setpoint = cartOutput * -maxOffset / 1000; // Calculate Angle PID setpoint based on Cart PID output

    pendulumPID.Compute();
    
    // Get elapsed time for data analysis purposes
    timeElapsed = millis() - startTime;

    // Move stepper motor
    startStepping(pendulumOutput, 10); // Moves set distance at a fixed speed

}


void swingup() { 
  delay(500);
  cartPosition = rawCartPosition * .03;

  // Moves cart to center
  while (abs(cartPosition) > 2) {
    cartPosition = rawCartPosition * .03;
    startStepping(-rawCartPosition, 100);
  }

  double oldangle = 0; // Used for finding if the pendulum is still swinging
  angle = rawPendulumPosition * .045;

  // Waits for pendlum to stop swinging
  while (oldangle != angle) {
    oldangle = angle;
    delay(1000);
    angle = rawPendulumPosition * .045;
  }
  rawPendulumPosition = 4000; // Once determined pendulum is still, resets position to 180 degrees
  Serial.println("angle set to 180");

  while (angle > 1 || angle < -1) { // Waits for pendulum to be brought back to the top
    delay(10);
    angle = rawPendulumPosition * .045;
    if (angle > 359 && angle < 361) {
      rawPendulumPosition = 0 + (angle-360)*(8000/360); // Resets angle to zero if rotated the wrong way
    }
    Serial.print(angle);
    Serial.print(", ");
    Serial.println("wating... ");
      }
  failstate = false; // resets failstate

}

void changingSetpoint() {
  if ((timeElapsed % 10000 <= 2) && timeElapsed > 1000) {
    Serial.println("setpoint changed to " + positions[state]);
    state = state + 1;
    if  (state == 4) {
      state = 0;
    }
    cartSetpoint = positions[state];
  }
}

void serialprint() {
    // Print data
    Serial.print(timeElapsed);
    Serial.print(",");
    Serial.print(setpoint);
    Serial.print(",");
    Serial.print(angle);
    Serial.print(",");
    Serial.print(pendulumOutput);
    Serial.print(",");
    Serial.print(cartPosition);
    Serial.print(",");
    Serial.println(cartOutput);
}

// Call this to begin a move
void startStepping(long numberOfSteps, long intervalMicros) {
  digitalWrite(DIR_PIN, numberOfSteps < 0 ? HIGH : LOW);
  stepsRemaining = abs(numberOfSteps) * 2; // 2 toggles per full step (HIGH + LOW)
  Timer1.setPeriod(intervalMicros);
  Timer1.start();
}

void moveStepper(double speed) {
    digitalWrite(DIR_PIN, speed < 0 ? HIGH : LOW);
    
    speed = abs(speed);
    for (int i = 0; i < speed; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(1);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(1);
    }
}

// This gets called automatically at each step interval
void stepperISR() {
  if (stepsRemaining > 0) {
    digitalWrite(STEP_PIN, stepState);
    stepState = !stepState;
    stepsRemaining--;
  } else {
    Timer1.stop();
    digitalWrite(STEP_PIN, LOW);
    stepState = false;
  }
}

void initialization() {
  // User calibration prompt for pendulum angle
    Serial.println("Let the pendulum hang down and press Enter to calibrate ANGLE to 180.");
    while (!Serial.available()) {}
    Serial.read();
    rawPendulumPosition = 4000;
    Serial.println("Pendulum angle calibrated!");

    // User calibration prompt for cart position
    Serial.println("Press Enter to calibrate the CART position to 0.");
    while (!Serial.available()) {}
    Serial.read();
    rawCartPosition = 0;
    Serial.println("Cart position calibrated!");
}

// Interrupt Service Routine for pendulum quadrature decoding

void encoderISR() {
  static uint8_t last = 0;
  uint8_t a = (PINE & (1 << 4)) ? 1 : 0;  // Read pin 2 (PE4)
  uint8_t b = (PINE & (1 << 5)) ? 1 : 0;  // Read pin 3 (PE5)
  uint8_t current = (a << 1) | b;

  uint8_t transition = (last << 2) | current;

  if (transition == 1 || transition == 7 || transition == 14 || transition == 8) {
    rawPendulumPosition++;
  } else if (transition == 2 || transition == 4 || transition == 13 || transition == 11) {
    rawPendulumPosition--;
  }

  last = current;
}

void cartEncoderISR() {
  static uint8_t last = 0;
  uint8_t a = (PIND & (1 << PD2)) ? 1 : 0;  // Read CART_ENCODER_A (e.g. pin D2)
  uint8_t b = (PIND & (1 << PD3)) ? 1 : 0;  // Read CART_ENCODER_B (e.g. pin D3)
  uint8_t current = (a << 1) | b;

  uint8_t transition = (last << 2) | current;

  if (transition == 1 || transition == 7 || transition == 14 || transition == 8) {
    rawCartPosition--;
  } else if (transition == 2 || transition == 4 || transition == 13 || transition == 11) {
    rawCartPosition++;
  }

  last = current;
}

