

// Including Libraries

#include <Wire.h>
#include <math.h>
#include "MedianFilterLib.h"
#include "src/MPU6050/MPU6050_6Axis_MotionApps20.h"
#include "src/button_panel/button_panel.h"

// Robot Properties
#define WHEEL_DIAMETER 36
#define TICKS_PER_ROUND_RIGHT 900
#define TICKS_PER_ROUND_LEFT 900

// Def for main modes
void test_encoder();
void test_buttons();
void test_buzzer();
void get_z_angle();
void get_distance();
void go_straight();


// Function Definitions
void BuzzerBeep(int count, int delayms = 50);

// Turn Directions
#define RIGHT_TURN 0
#define LEFT_TURN 1
#define TURN_BACK 2
#define FORWARD_CELL 3
#define INIT_STATE 4
#define FASTRUN_CELL 5

// Speed Run Definitions
#define FASTRUN_GO_FORWARD 0
#define FASTRUN_TURNED_GO_FORWARD 1
#define FASTRUN_GO_FORWARD_AND_TURN_RIGHT 2
#define FASTRUN_TURN_RIGHT 3
#define FASTRUN_GO_FORWARD_AND_TURN_LEFT 4
#define FASTRUN_TURN_LEFT 5
#define FASTRUN_CURVE_TURN_INIT 6
#define FASTRUN_GO_BACK 7

// Wall Follow Definitions
#define MAXIMUM_ACCEL_CURVE_COUNT 40

// Wall Sides
#define LEFT_SIDE 0
#define RIGHT_SIDE 1