package com.frcteam3255.joystick;

import com.frcteam3255.preferences.SN_IntPreference;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * Custom Joystick for the Logitech F310
 * <p>
 * When the switch on the back of the controller is flipped to the X setting,
 * the triggers register as analog inputs. This custom joystick utilizes those
 * analog triggers.
 */

public class SN_F310Gamepad extends Joystick {
	/** Joystick Button 1 */
	public JoystickButton btn_A = new JoystickButton(this, 1);
	/** Joystick Button 2 */
	public JoystickButton btn_B = new JoystickButton(this, 2);
	/** Joystick Button 3 */
	public JoystickButton btn_X = new JoystickButton(this, 3);
	/** Joystick Button 4 */
	public JoystickButton btn_Y = new JoystickButton(this, 4);
	/** Joystick Button 5 */
	public JoystickButton btn_LBump = new JoystickButton(this, 5);
	/** Joystick Button 6 */
	public JoystickButton btn_RBump = new JoystickButton(this, 6);
	/** Joystick Button 7 */
	public JoystickButton btn_Back = new JoystickButton(this, 7);
	/** Joystick Button 8 */
	public JoystickButton btn_Start = new JoystickButton(this, 8);
	/** Joystick Button 9 */
	public JoystickButton btn_LStick = new JoystickButton(this, 9);
	/** Joystick Button 10 */
	public JoystickButton btn_RStick = new JoystickButton(this, 10);

	// POV Buttons
	public POVButton POV_North = new POVButton(this, 0);
	public POVButton POV_NorthEast = new POVButton(this, 45);
	public POVButton POV_East = new POVButton(this, 90);
	public POVButton POV_SouthEast = new POVButton(this, 135);
	public POVButton POV_South = new POVButton(this, 180);
	public POVButton POV_SouthWest = new POVButton(this, 225);
	public POVButton POV_West = new POVButton(this, 270);
	public POVButton POV_NorthWest = new POVButton(this, 315);

	private static final int AXIS_ARCADE_MOVE = 1;
	private static final int AXIS_ARCADE_ROTATE = 4;
	private static final int AXIS_ARCADE_STRAFE = 0;

	private static final int AXIS_TANK_LEFT = 1;
	private static final int AXIS_TANK_RIGHT = 5;

	private static final int AXIS_LS_X = 0;
	private static final int AXIS_LS_Y = 1;
	private static final int AXIS_LT = 2;
	private static final int AXIS_RT = 3;
	private static final int AXIS_RS_X = 4;
	private static final int AXIS_RS_Y = 5;

	private static final double TRIGGER_PRESS_THRESHOLD = 0.5;

	public Trigger trig_LT = new Trigger(() -> getAxisLT() > TRIGGER_PRESS_THRESHOLD);
	public Trigger trig_RT = new Trigger(() -> getAxisRT() > TRIGGER_PRESS_THRESHOLD);

	/**
	 * Logitech F310 Gamepad
	 *
	 * @param port
	 *            The port on the Driver Station that the joystick is plugged into.
	 */
	public SN_F310Gamepad(final int port) {
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
	 * @return position value of RawAxis({@value #AXIS_LS_X})
	 */
	public double getAxisLSX() {
		return getRawAxis(AXIS_LS_X);
	}

	/**
	 * @return inverted position value of RawAxis({@value #AXIS_LS_Y})
	 */
	public double getAxisLSY() {
		return -getRawAxis(AXIS_LS_Y);
	}

	/**
	 * @return position value of RawAxis({@value #AXIS_LT})
	 */
	public double getAxisLT() {
		return getRawAxis(AXIS_LT);
	}

	/**
	 * @return position value of RawAxis({@value #AXIS_RT})
	 */
	public double getAxisRT() {
		return getRawAxis(AXIS_RT);
	}

	/**
	 * @return position value of RawAxis({@value #AXIS_RS_X})
	 */
	public double getAxisRSX() {
		return getRawAxis(AXIS_RS_X);
	}

	/**
	 * @return inverted position value of RawAxis({@value #AXIS_RS_Y})
	 */
	public double getAxisRSY() {
		return -getRawAxis(AXIS_RS_Y);
	}

	/**
	 * @param axisId
	 *            SN_IntPreference
	 * @return position value of axis
	 */
	public double getAxisVar(SN_IntPreference axisId) {

		return getRawAxis(axisId.getValue());
	}

}
