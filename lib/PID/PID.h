#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION 1.2.3

#include <cmath>

class PID {
public:
  // Constants used in some of the functions below
#define AUTOMATIC 1
#define MANUAL    0
#define DIRECT    0
#define REVERSE   1
#define P_ON_M    0
#define P_ON_E    1

  // Commonly used functions **************************************************************************

  // Constructor: links the PID to Input, Output, and Setpoint.
  // Initial tuning parameters and proportional mode are set here.
  PID(double* Input, double* Output, double* Setpoint,
      double Kp, double Ki, double Kd, int POn, int ControllerDirection);

  // Constructor overload: defaults to P_ON_E (proportional on error)
  PID(double* Input, double* Output, double* Setpoint,
      double Kp, double Ki, double Kd, int ControllerDirection);

  void SetMode(int Mode); // Sets PID to Manual (0) or Automatic (non-0)
  bool Compute(); // Performs the PID calculation; call every loop() cycle
  void SetOutputLimits(double Min, double Max); // Clamps output to a specific range (default 0–255)

  // Available but not commonly used functions ********************************************************

  void SetTunings(double Kp, double Ki, double Kd); // Change tunings at runtime
  void SetTunings(double Kp, double Ki, double Kd, int POn); // Overload to also set proportional mode

  void SetControllerDirection(int Direction); // DIRECT: output↑ when error↑; REVERSE: opposite
  void SetSampleTime(int NewSampleTime); // Sets calculation frequency in milliseconds (default 100)

  // Resets the integral accumulator and derivative state.
  // Use this to clear integral windup when switching setpoints or recovering from saturation.
  void ResetIntegral();

  // D-term low-pass filter (EMA). alpha in (0,1]: lower = smoother, higher = faster.
  // Filters the input signal before computing dInput, reducing derivative kick from sensor noise.
  void SetDTermFilter(bool enable, double alpha = 0.1);

  // Suppress integral accumulation when |error| > maxError.
  // Prevents windup during large transients; integral resumes once error falls inside the band.
  void SetConditionalIntegral(bool enable, double maxError);

  // Display / query functions ************************************************************************

  [[nodiscard]] double GetKp() const;
  [[nodiscard]] double GetKi() const;
  [[nodiscard]] double GetKd() const;
  [[nodiscard]] int GetMode() const;
  [[nodiscard]] int GetDirection() const;

private:
  void Initialize();

  double dispKp{}; // User-entered tuning parameters (for display)
  double dispKi{};
  double dispKd{};

  double kp{}; // Scaled proportional gain
  double ki{}; // Scaled integral gain
  double kd{}; // Scaled derivative gain

  int controllerDirection{};
  int pOn{};

  double* myInput;
  double* myOutput;
  double* mySetpoint;

  unsigned long lastTime{};
  double outputSum{};
  double lastInput{};

  unsigned long SampleTime{};
  double outMin{};
  double outMax{};
  bool inAuto{};
  bool pOnE{};

  // First-execution guard: seeds lastInput on the very first Compute() call
  // to avoid a large dInput spike from an uninitialised lastInput of 0.
  bool firstRun{true};

  // Error sign-change tracking: resets integral when error crosses zero
  // to prevent integrator overshoot during setpoint approach.
  double lastError{};

  // D-term EMA filter
  bool dFilterEnabled{};
  double dFilterAlpha{0.1};
  double filteredInput{};

  // Conditional integral: suppresses accumulation when |error| > threshold
  bool conditionalIEnabled{};
  double conditionalIMaxError{};
};

#endif
