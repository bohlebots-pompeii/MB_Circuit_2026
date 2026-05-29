/**********************************************************************************************
 * Arduino PID Library - Version 1.2.3
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * modified by Julius Gerhardus
 *
 * This Library is licensed under the MIT License
 *
 * huhu im a commit
 **********************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <cmath>
#include <PID.h>

/*Constructor (...)*********************************************************
 * Full constructor: specifies proportional mode explicitly.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
         const double Kp, const double Ki, const double Kd,
         const int POn, const int ControllerDirection) {
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = false;

  SetOutputLimits(0, 255); // Default: Arduino PWM range

  SampleTime = 100; // Default sample time: 100 ms

  SetControllerDirection(ControllerDirection);
  SetTunings(Kp, Ki, Kd, POn);

  lastTime = millis() - SampleTime;
}

/*Constructor (...)*********************************************************
 * Backwards-compatible overload: defaults to P_ON_E.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
         double Kp, double Ki, double Kd, int ControllerDirection)
  : PID::PID(Input, Output, Setpoint, Kp, Ki, Kd, P_ON_E, ControllerDirection) {}


/* Compute() **********************************************************************
 * Call every loop() cycle. Returns true when a new output was computed.
 **********************************************************************************/
bool PID::Compute() {
  if (!inAuto) return false;

  const unsigned long now = millis();
  const unsigned long timeChange = now - lastTime;

  if (timeChange >= SampleTime) {
    const double input = *myInput;
    const double error = *mySetpoint - input;

    // On the very first Compute() call after going auto, seed lastInput and
    // lastError from live values so dInput and the zero-crossing check don't
    // fire spuriously from uninitialised zeros.
    if (firstRun) {
      lastInput = input;
      lastError = error;
      filteredInput = input;
      firstRun = false;
    }

    // Zero-crossing reset: when the error changes sign the setpoint has been
    // crossed — clear the integrator to prevent accumulated windup from
    // causing overshoot on the other side.
    if ((error > 0.0) != (lastError > 0.0)) {
      outputSum = 0.0;
    }

    // D-term EMA filter: smooth the input before computing the derivative to
    // reduce kick from high-frequency sensor noise.  alpha=1 disables filtering.
    double inputForD = input;
    if (dFilterEnabled) {
      filteredInput = dFilterAlpha * input + (1.0 - dFilterAlpha) * filteredInput;
      inputForD = filteredInput;
    }

    const double dInput = inputForD - lastInput;

    // Conditional integral: suspend accumulation during large transients so
    // windup cannot build up while the output is already saturated.
    const bool accumulateI = !conditionalIEnabled
      || (std::abs(error) <= conditionalIMaxError);
    if (accumulateI) {
      outputSum += ki * error;
    }

    // Proportional on Measurement (P_ON_M): subtract proportional from accumulator
    if (!pOnE) outputSum -= kp * dInput;

    // Clamp integrator before adding proportional term (anti-windup)
    if (outputSum > outMax) outputSum = outMax;
    else if (outputSum < outMin) outputSum = outMin;

    // Proportional on Error (P_ON_E): add proportional to output directly
    double output = pOnE ? kp * error : 0.0;

    // Complete PID output
    output += outputSum - kd * dInput;

    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;
    *myOutput = output;

    lastInput = inputForD;
    lastError = error;
    lastTime = now;
    return true;
  }

  return false;
}

/* SetTunings(...)*************************************************************
 * Adjust tunings at runtime. Gains must be non-negative.
 * Returns without changing anything if any gain is negative.
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd, int POn) {
  if (Kp < 0 || Ki < 0 || Kd < 0) return;

  pOn = POn;
  pOnE = (POn == P_ON_E);

  dispKp = Kp;
  dispKi = Ki;
  dispKd = Kd;

  const double SampleTimeInSec = static_cast<double>(SampleTime) / 1000.0;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;

  if (controllerDirection == REVERSE) {
    kp = -kp;
    ki = -ki;
    kd = -kd;
  }
}

/* SetTunings(...)*************************************************************
 * Overload: uses the previously stored POn setting.
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd) {
  SetTunings(Kp, Ki, Kd, pOn);
}

/* SetSampleTime(...) *********************************************************
 * Sets the calculation period in milliseconds.
 * Rescales ki and kd so existing behavior is preserved at the new rate.
 ******************************************************************************/
void PID::SetSampleTime(const int NewSampleTime) {
  if (NewSampleTime <= 0) return;

  const double ratio = static_cast<double>(NewSampleTime)
    / static_cast<double>(SampleTime);
  ki *= ratio;
  kd /= ratio;
  SampleTime = static_cast<unsigned long>(NewSampleTime);
}

/* SetOutputLimits(...)********************************************************
 * Clamps the output and integrator to [Min, Max].
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max) {
  if (Min >= Max) return;
  outMin = Min;
  outMax = Max;

  if (inAuto) {
    if (*myOutput > outMax) *myOutput = outMax;
    else if (*myOutput < outMin) *myOutput = outMin;

    if (outputSum > outMax) outputSum = outMax;
    else if (outputSum < outMin) outputSum = outMin;
  }
}

/* SetMode(...)****************************************************************
 * Switch between Manual (0) and Automatic (non-zero).
 * Initializes bumplessly when transitioning from manual to auto.
 ******************************************************************************/
void PID::SetMode(int Mode) {
  const bool newAuto = (Mode == AUTOMATIC);
  if (newAuto && !inAuto) {
    PID::Initialize();
  }
  inAuto = newAuto;
}

/* Initialize()****************************************************************
 * Prepares for a bumpless manual→auto transition by seeding the integrator
 * with the current output and snapshotting the current input.
 ******************************************************************************/
void PID::Initialize() {
  outputSum = *myOutput;
  lastInput = *myInput;
  filteredInput = *myInput;
  firstRun = true; // Re-seed on the next Compute() call

  if (outputSum > outMax) outputSum = outMax;
  else if (outputSum < outMin) outputSum = outMin;
}

/* ResetIntegral()************************************************************
 * Zeroes the integral accumulator and derivative state.
 *
 * Use this when:
 *   - The setpoint changes significantly and windup has built up
 *   - The process recovers from a disturbance, and you want a clean start
 *   - Switching between control zones or operating modes
 *
 * Unlike Initialize(), this does NOT seed from the current output —
 * it performs a hard reset. The next Compute() call will start fresh.
 ******************************************************************************/
void PID::ResetIntegral() {
  outputSum = 0.0;
  lastInput = *myInput; // Prevent a dInput spike on the next tick
  filteredInput = *myInput;
  firstRun = false; // lastInput is now valid; no need to re-seed
}

/* SetDTermFilter(...)*********************************************************
 * Enables an exponential moving average (EMA) on the input signal before
 * computing dInput. Reduces derivative kick from sensor noise.
 * alpha in (0, 1]: 0.1 = heavy smoothing, 1.0 = no filter (pass-through).
 ******************************************************************************/
void PID::SetDTermFilter(bool enable, double alpha) {
  dFilterEnabled = enable;
  dFilterAlpha = (alpha > 0.0 && alpha <= 1.0) ? alpha : 0.1;
}

/* SetConditionalIntegral(...)************************************************
 * When enabled, the integral term only accumulates when |error| <= maxError.
 * This prevents windup during large transients (e.g. after a big setpoint
 * step) while still allowing the integrator to work near the target.
 ******************************************************************************/
void PID::SetConditionalIntegral(bool enable, double maxError) {
  conditionalIEnabled = enable;
  conditionalIMaxError = maxError;
}

/* SetControllerDirection(...)*************************************************
 * DIRECT:  +output leads to +input  (most processes)
 * REVERSE: +output leads to -input  (e.g. cooling systems)
 ******************************************************************************/
void PID::SetControllerDirection(int Direction) {
  if (inAuto && Direction != controllerDirection) {
    kp = -kp;
    ki = -ki;
    kd = -kd;
  }
  controllerDirection = Direction;
}

/* Status / display functions *************************************************/
double PID::GetKp() const { return dispKp; }
double PID::GetKi() const { return dispKi; }
double PID::GetKd() const { return dispKd; }
int PID::GetMode() const { return inAuto ? AUTOMATIC : MANUAL; }
int PID::GetDirection() const { return controllerDirection; }
