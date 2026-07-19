#include <stdint.h>
#include  <stdbool.h>

extern volatile float rpm;
extern volatile float angle;
extern volatile float encoder_angle;
extern volatile bool motorEnable;
extern volatile float pitch;
// For a PID controller
extern volatile float Kp;
extern volatile float Ki;
extern volatile float Kd;
