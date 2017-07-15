// note on plain values:
// buttons are either 0 or 1
// button axes go from 0 to -1
// stick axes go from 0 to +/-1


// TODO: proper ROS namespace name?
namespace xbox360joy

{

const int XBOX360_BUTTON_A                = 0;
const int XBOX360_BUTTON_B                = 1;
const int XBOX360_BUTTON_X                = 2;
const int XBOX360_BUTTON_Y                = 3;
const int XBOX360_BUTTON_LB               = 4;
const int XBOX360_BUTTON_RB               = 5;
const int XBOX360_BUTTON_BACK             = 6;
const int XBOX360_BUTTON_START            = 7;
const int XBOX360_BUTTON_POWER            = 8;
const int XBOX360_BUTTON_STICK_LEFT       = 9;
const int XBOX360_BUTTON_STICK_RIGHT      = 10;

const int XBOX360_AXIS_STICK_LEFT_LEFTWARDS   = 0;
const int XBOX360_AXIS_STICK_LEFT_UPWARDS     = 1;
const int XBOX360_AXIS_LEFT_TRIGGER           = 2;
const int XBOX360_AXIS_STICK_RIGHT_LEFTWARDS  = 3;
const int XBOX360_AXIS_STICK_RIGHT_UPWARDS    = 4;
const int XBOX360_AXIS_RIGHT_TRIGGER          = 5;
const int XBOX360_AXIS_CROSS_LEFTWARDS        = 6;
const int XBOX360_AXIS_CROSS_UPWARDS          = 7;

} // end namespace xbox360joy
