# SuperCORE

SuperCORE is a vendor library for FRC projects. SuperCORE has several uses:

* handles controllers
* modified RobotPreferences
* custom utilities 

These are tasks that need to be done every year, so instead of just rewriting all the code (or copying it from last year's project) SuperCORE was created as a vendor library to make it as quick and easy as possible to get a new robot functional.

## Installation Instructions:

1. In Visual Studio Code press Ctrl + shift + p
2. Type "Manage vendor libraries" into the prompt and hit enter
3. Select "Install new libraries (online)"
4. Copy and paste the following link into the prompt: `https://frcteam3255.github.io/SuperCORE/releases/com/frcteam3255/supercore/SuperCORE-latest.json`

## Supported Controlers:

* Logitech F310 Gamepad
    * SN_DualActionStick
    * SN_F310Gamepad
* Logitech Extreme 3D Pro Joystick
    * SN_Extreme3DStick
* Custom Arduino Switchboard
    * SN_SwitchboardStick

## Utilities

* Alternative debug output
    * SN_Debug
    * All SN_Debug message output can be disabled very easily
* Linear Interpolation
    * SN_Math
* Turn CSVs into Motion Profiles
    * SN_MotionProfile

## Robot Preferences
 Using the SuperCORE preferences gives the ability to swtich between using the hardcoded values and the Network Tables values.

## Competetion
 It is recommended during a competition that:
* all SN_Debug messages be disabled, making other important error messages more visible.
* hardcoded Robot Preferences are used.
    * This makes the behavior of the Robot based only on the code that the robot is running, and not on any other external factors, like the system that is running the drivers station software

## Documentation

[Java Docs](https://frcteam3255.github.io/SuperCORE/releases/com/frcteam3255/supercore/javadoc-latest/)
