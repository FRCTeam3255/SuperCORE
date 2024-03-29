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
 * Custom 12 Button Joystick set up for Logitech Extreme 3D Joystick
 */
public class SN_Extreme3DStick extends Joystick {
	public JoystickButton btn_1 = new JoystickButton(this, 1);
	public JoystickButton btn_2 = new JoystickButton(this, 2);
	public JoystickButton btn_3 = new JoystickButton(this, 3);
	public JoystickButton btn_4 = new JoystickButton(this, 4);
	public JoystickButton btn_5 = new JoystickButton(this, 5);
	public JoystickButton btn_6 = new JoystickButton(this, 6);
	public JoystickButton btn_7 = new JoystickButton(this, 7);
	public JoystickButton btn_8 = new JoystickButton(this, 8);
	public JoystickButton btn_9 = new JoystickButton(this, 9);
	public JoystickButton btn_10 = new JoystickButton(this, 10);
	public JoystickButton btn_11 = new JoystickButton(this, 11);
	public JoystickButton btn_12 = new JoystickButton(this, 12);

	// POV Buttons
	public POVButton POV_North = new POVButton(this, 0);
	public POVButton POV_NorthEast = new POVButton(this, 45);
	public POVButton POV_East = new POVButton(this, 90);
	public POVButton POV_SouthEast = new POVButton(this, 135);
	public POVButton POV_South = new POVButton(this, 180);
	public POVButton POV_SouthWest = new POVButton(this, 225);
	public POVButton POV_West = new POVButton(this, 270);
	public POVButton POV_NorthWest = new POVButton(this, 315);

	private static final int AXIS_X = 0;
	private static final int AXIS_Y = 1;
	private static final int AXIS_TWIST = 2;
	private static final int AXIS_DIAL = 3;

	/**
	 * Logitech Extreme 3D Pro Joystick with 12 buttons
	 *
	 * @param port
	 *            The port on the Driver Station that the joystick is plugged into.
	 */
	public SN_Extreme3DStick(final int port) {
		super(port);
	}

	/**
	 * @return position value of RawAxis({@value #AXIS_X})
	 */
	public double getXAxis() {
		return getRawAxis(AXIS_X);
	}

	/**
	 * @return inverted position value of RawAxis({@value #AXIS_Y})
	 */
	public double getYAxis() {
		return -getRawAxis(AXIS_Y);
	}

	/**
	 * @return position Value of RawAxis({@value #AXIS_TWIST})
	 */
	public double getTwistAxis() {
		return getRawAxis(AXIS_TWIST);
	}

	/**
	 * @return inverted position value of RawAxis({@value #AXIS_DIAL})
	 */
	public double getDialAxis() {
		return (((getRawAxis(AXIS_DIAL) * -1) + 1) / 2);
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
