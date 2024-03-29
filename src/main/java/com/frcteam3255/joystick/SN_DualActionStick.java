/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.frcteam3255.joystick;

import com.frcteam3255.preferences.SN_IntPreference;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * Custom 12 Button Joystick set up for Logitech F310 Gamepad
 * <p>
 * Adds custom axis return methods such as {@link #getArcadeMove()} and
 * {@link #getArcadeRotate()}
 */
public class SN_DualActionStick extends Joystick {
	/** Joystick Button 1 */
	public JoystickButton btn_X = new JoystickButton(this, 1);
	/** Joystick Button 2 */
	public JoystickButton btn_A = new JoystickButton(this, 2);
	/** Joystick Button 3 */
	public JoystickButton btn_B = new JoystickButton(this, 3);
	/** Joystick Button 4 */
	public JoystickButton btn_Y = new JoystickButton(this, 4);
	/** Joystick Button 5 */
	public JoystickButton btn_LBump = new JoystickButton(this, 5);
	/** Joystick Button 6 */
	public JoystickButton btn_RBump = new JoystickButton(this, 6);
	/** Joystick Button 7 */
	public JoystickButton btn_LTrig = new JoystickButton(this, 7);
	/** Joystick Button 8 */
	public JoystickButton btn_RTrig = new JoystickButton(this, 8);
	/** Joystick Button 9 */
	public JoystickButton btn_Back = new JoystickButton(this, 9);
	/** Joystick Button 10 */
	public JoystickButton btn_Start = new JoystickButton(this, 10);
	/** Joystick Button 11 */
	public JoystickButton btn_LStick = new JoystickButton(this, 11);
	/** Joystick Button 12 */
	public JoystickButton btn_RStick = new JoystickButton(this, 12);

	// POV Buttons
	public POVButton POV_North = new POVButton(this, 0);
	public POVButton POV_NorthEast = new POVButton(this, 45);
	public POVButton POV_East = new POVButton(this, 90);
	public POVButton POV_SouthEast = new POVButton(this, 135);
	public POVButton POV_South = new POVButton(this, 180);
	public POVButton POV_SouthWest = new POVButton(this, 225);
	public POVButton POV_West = new POVButton(this, 270);
	public POVButton POV_NorthWest = new POVButton(this, 315);

	// Axes
	private static final int AXIS_ARCADE_MOVE = 1;
	private static final int AXIS_ARCADE_ROTATE = 2;
	private static final int AXIS_ARCADE_STRAFE = 0;

	private static final int AXIS_TANK_LEFT = 1;
	private static final int AXIS_TANK_RIGHT = 5;

	private static final int AXIS_LEFT_STICK_X = 0;
	private static final int AXIS_LEFT_STICK_Y = 1;
	private static final int AXIS_RIGHT_STICK_X = 2;
	private static final int AXIS_RIGHT_STICK_Y = 3;

	/**
	 * Logitech F310 Gamepad with 12 Buttons and custom Axes
	 *
	 * @param port
	 *            The port on the Driver Station that the joystick is plugged into.
	 */
	public SN_DualActionStick(final int port) {
		super(port);
	}

	// Arcade Drive
	/**
	 * @return inverted position value of RawAxis({@value #AXIS_ARCADE_MOVE})
	 */
	public double getArcadeMove() {
		return -getRawAxis(AXIS_ARCADE_MOVE);
	}

	/**
	 * @return position value of RawAxis({@value #AXIS_ARCADE_ROTATE})
	 */
	public double getArcadeRotate() {
		return getRawAxis(AXIS_ARCADE_ROTATE);
	}

	/**
	 * @return inverted position value of RawAxis({@value #AXIS_ARCADE_STRAFE})
	 */
	public double getArcadeStrafe() {
		return getRawAxis(AXIS_ARCADE_STRAFE);
	}

	// Tank Drive
	/**
	 * @return inverted position value of RawAxis({@value #AXIS_TANK_LEFT})
	 */
	public double getTankLeft() {
		return -getRawAxis(AXIS_TANK_LEFT);
	}

	/**
	 * @return inverted position value of RawAxis({@value #AXIS_TANK_RIGHT})
	 */
	public double getTankRight() {
		return -getRawAxis(AXIS_TANK_RIGHT);
	}

	// Get Axis

	/**
	 * @return position value of RawAxis({@value #AXIS_LEFT_STICK_X})
	 */
	public double getLeftStickX() {
		return getRawAxis(AXIS_LEFT_STICK_X);
	}

	/**
	 * @return inverted position value of RawAxis({@value #AXIS_LEFT_STICK_Y})
	 */
	public double getLeftStickY() {
		return -getRawAxis(AXIS_LEFT_STICK_Y);
	}

	/**
	 * @return position value of RawAxis({@value #AXIS_RIGHT_STICK_X})
	 */
	public double getRightStickX() {
		return getRawAxis(AXIS_RIGHT_STICK_X);
	}

	/**
	 * @return inverted position value of RawAxis({@value #AXIS_RIGHT_STICK_Y})
	 */
	public double getRightStickY() {
		return -getRawAxis(AXIS_RIGHT_STICK_Y);
	}
	/**
	 * @param axisId
	 *            id number of controller axis, viewable in this file if needed
	 * @return position value of RawAxis(axisId)
	 */
	public double getAxisVar(SN_IntPreference axisId) {
		return getRawAxis(axisId.getValue());
	}
}
