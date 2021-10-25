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

* Alternative debug output; replaces System.out.println()
    * SN_Debug:
* Linear Interpolation
    * SN_Math
* Turn CSVs into Motion Profiles
    * SN_MotionProfile


## Documentation

[Java Docs](https://frcteam3255.github.io/SuperCORE/releases/com/frcteam3255/supercore/javadoc-latest/)
