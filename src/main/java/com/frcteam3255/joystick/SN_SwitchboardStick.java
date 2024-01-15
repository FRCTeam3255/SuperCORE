/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.frcteam3255.joystick;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * Custom 32 Button Joystick set up for the custom-made SuperNURDs Switchboard
 */
public class SN_SwitchboardStick extends Joystick {
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
	public JoystickButton btn_13 = new JoystickButton(this, 13);
	public JoystickButton btn_14 = new JoystickButton(this, 14);
	public JoystickButton btn_15 = new JoystickButton(this, 15);
	public JoystickButton btn_16 = new JoystickButton(this, 16);
	public JoystickButton btn_17 = new JoystickButton(this, 17);
	public JoystickButton btn_18 = new JoystickButton(this, 18);
	public JoystickButton btn_19 = new JoystickButton(this, 19);
	public JoystickButton btn_20 = new JoystickButton(this, 20);
	public JoystickButton btn_21 = new JoystickButton(this, 21);
	public JoystickButton btn_22 = new JoystickButton(this, 22);
	public JoystickButton btn_23 = new JoystickButton(this, 23);
	public JoystickButton btn_24 = new JoystickButton(this, 24);
	public JoystickButton btn_25 = new JoystickButton(this, 25);
	public JoystickButton btn_26 = new JoystickButton(this, 26);
	public JoystickButton btn_27 = new JoystickButton(this, 27);
	public JoystickButton btn_28 = new JoystickButton(this, 28);
	public JoystickButton btn_29 = new JoystickButton(this, 29);
	public JoystickButton btn_30 = new JoystickButton(this, 30);
	public JoystickButton btn_31 = new JoystickButton(this, 31);
	public JoystickButton btn_32 = new JoystickButton(this, 32);

	/**
	 * Logitech Extreme 3D Pro Joystick with 12 buttons
	 *
	 * @param port
	 *            The port on the Driver Station that the joystick is plugged into.
	 */
	public SN_SwitchboardStick(final int port) {
		super(port);
	}
}
