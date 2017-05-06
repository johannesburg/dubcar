// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1


// TODO: proper ROS namespace name?
namespace ps3joy

{

const int PS3_BUTTON_SELECT           = 0;
const int PS3_BUTTON_STICK_LEFT       = 1;
const int PS3_BUTTON_STICK_RIGHT      = 2;
const int PS3_BUTTON_START            = 3;
const int PS3_BUTTON_CROSS_UP         = 4;
const int PS3_BUTTON_CROSS_RIGHT      = 5;
const int PS3_BUTTON_CROSS_DOWN       = 6;
const int PS3_BUTTON_CROSS_LEFT       = 7;
const int PS3_BUTTON_REAR_LEFT_2      = 8;
const int PS3_BUTTON_REAR_RIGHT_2     = 9;
const int PS3_BUTTON_REAR_LEFT_1      = 10;
const int PS3_BUTTON_REAR_RIGHT_1     = 11;
const int PS3_BUTTON_ACTION_TRIANGLE  = 12;
const int PS3_BUTTON_ACTION_CIRCLE    = 13;
const int PS3_BUTTON_ACTION_CROSS     = 14;
const int PS3_BUTTON_ACTION_SQUARE    = 15;
const int PS3_BUTTON_PAIRING          = 16;

const int PS3_AXIS_STICK_LEFT_LEFTWARDS   = 0;
const int PS3_AXIS_STICK_LEFT_UPWARDS     = 1;
const int PS3_AXIS_STICK_RIGHT_LEFTWARDS  = 2;
const int PS3_AXIS_STICK_RIGHT_UPWARDS    = 3;
const int PS3_AXIS_BUTTON_CROSS_UP        = 4;
const int PS3_AXIS_BUTTON_CROSS_RIGHT     = 5;
const int PS3_AXIS_BUTTON_CROSS_DOWN      = 6;
const int PS3_AXIS_BUTTON_CROSS_LEFT      = 7;
const int PS3_AXIS_BUTTON_REAR_LEFT_2     = 8;
const int PS3_AXIS_BUTTON_REAR_RIGHT_2    = 9;
const int PS3_AXIS_BUTTON_REAR_LEFT_1     = 10;
const int PS3_AXIS_BUTTON_REAR_RIGHT_1    = 11;
const int PS3_AXIS_BUTTON_ACTION_TRIANGLE = 12;
const int PS3_AXIS_BUTTON_ACTION_CIRCLE   = 13;
const int PS3_AXIS_BUTTON_ACTION_CROSS    = 14;
const int PS3_AXIS_BUTTON_ACTION_SQUARE   = 15;
const int PS3_AXIS_ACCELEROMETER_LEFT     = 16;
const int PS3_AXIS_ACCELEROMETER_FORWARD  = 17;
const int PS3_AXIS_ACCELEROMETER_UP       = 18;
const int PS3_AXIS_GYRO_YAW               = 19;

} // end namespace ps3joy
