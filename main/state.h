#include <stdint.h>
#include  <stdbool.h>

extern volatile float rpm;
extern volatile float angle;
extern volatile float encoder_angle;
extern volatile bool motorEnable;
extern volatile float pitch;
extern volatile float pitch_actual;
extern volatile bool pitchMotorEnable;
extern volatile bool pitchCalibrated;
extern volatile bool pitchHomingFailed;
extern volatile bool trigger;
void set_trigger(bool enable);
extern volatile bool auto_fire;
// For a PID controller
extern volatile float Kp;
extern volatile float Ki;
extern volatile float Kd;
extern volatile float PitchKp;
extern volatile float PitchKi;
extern volatile float PitchKd;
