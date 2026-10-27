#pragma once

// App mode names avoid PID_v1 macros.
enum Mode {
  REMOTE_MODE,
  DISTANCE_MODE,
  CALIBRATION_MODE,
  PID_TUNING_MODE
};

enum Motion {
  STOPPED,
  FORWARD,
  BACKWARD,
  TURN_LEFT,
  TURN_RIGHT
};
