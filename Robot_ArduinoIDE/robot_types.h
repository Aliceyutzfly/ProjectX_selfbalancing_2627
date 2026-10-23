#pragma once
// Application mode names avoid the PID library's MANUAL macro.
enum Mode { REMOTE_MODE, DISTANCE_MODE, CALIBRATION_MODE };
enum Motion { STOPPED, FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT };
