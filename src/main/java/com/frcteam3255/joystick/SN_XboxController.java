package com.frcteam3255.joystick;

import java.util.function.DoubleSupplier;

import org.wpilib.math.util.MathUtil;
import org.wpilib.command2.button.CommandGamepad;
import org.wpilib.command2.button.Trigger;

/**
 * Custom class for Xbox Controller use
 */
public class SN_XboxController extends CommandGamepad {

	public Trigger btn_A = southFace();
	public Trigger btn_B = eastFace();
	public Trigger btn_X = westFace();
	public Trigger btn_Y = northFace();

	public Trigger btn_LeftBumper = leftBumper();
	public Trigger btn_RightBumper = rightBumper();

	public Trigger btn_Start = start();
	public Trigger btn_Back = back();

	public Trigger btn_LeftStick = leftStick();
	public Trigger btn_RightStick = rightStick();

	public Trigger btn_North = dpadUp();
	public Trigger btn_East = dpadRight();
	public Trigger btn_South = dpadDown();
	public Trigger btn_West = dpadLeft();

	private double TRIGGER_PRESS_THRESHOLD = 0.5;

	public Trigger btn_LeftTrigger = leftTrigger(TRIGGER_PRESS_THRESHOLD);
	public Trigger btn_RightTrigger = rightTrigger(TRIGGER_PRESS_THRESHOLD);

	private double LEFT_DEADBAND = 0.1;
	private double RIGHT_DEADBAND = 0.1;

	public DoubleSupplier axis_LeftX = () -> MathUtil.applyDeadband(super.getLeftX(), LEFT_DEADBAND);
	public DoubleSupplier axis_LeftY = () -> MathUtil.applyDeadband(-super.getLeftY(), LEFT_DEADBAND);
	public DoubleSupplier axis_RightX = () -> MathUtil.applyDeadband(super.getRightX(), RIGHT_DEADBAND);
	public DoubleSupplier axis_RightY = () -> MathUtil.applyDeadband(-super.getRightY(), RIGHT_DEADBAND);
	public DoubleSupplier axis_LeftTrigger = () -> super.getLeftTriggerAxis();
	public DoubleSupplier axis_RightTrigger = () -> super.getRightTriggerAxis();

	/**
	 * Set the value that the triggers must be at to register as true while being
	 * used as a button. A value of zero will make the trigger act as a hair trigger
	 * (will activate at the slightest touch) and a value of one will make the
	 * trigger be held all the way down to activate.
	 * <p>
	 * Default value is 0.5
	 *
	 * @param threshold
	 *            for trigger button
	 */
	public void setTriggerPressThreshold(double threshold) {
		TRIGGER_PRESS_THRESHOLD = Math.clamp(threshold, 0, 0.99);
	}

	/**
	 * Set the deadband for the left joystick.
	 * <p>
	 * Default value is 0.1
	 *
	 * @param deadband
	 *            to set for the left joystick
	 */
	public void setLeftDeadband(double deadband) {
		LEFT_DEADBAND = deadband;
	}

	/**
	 * Set the deadband for the right joystick.
	 * <p>
	 * Default value is 0.1
	 *
	 * @param deadband
	 *            to set for the right joystick
	 */
	public void setRightDeadband(double deadband) {
		RIGHT_DEADBAND = deadband;
	}

	/**
	 * Construct an instance of an SN_XboxController.
	 *
	 * @param port
	 *            The port index on the Driver Station that the controller is
	 *            plugged into.
	 * @param leftDeadband
	 *            The deadband for the left joystick.
	 * @param rightDeadband
	 *            The deadband for the right joystick.
	 */
	public SN_XboxController(int port, double leftDeadband, double rightDeadband) {
		super(port);
		LEFT_DEADBAND = leftDeadband;
		RIGHT_DEADBAND = rightDeadband;
	}

	/**
	 * Construct an instance of an SN_XboxController.
	 *
	 * @param port
	 *            The port index on the Driver Station that the controller is
	 *            plugged into.
	 */
	public SN_XboxController(int port) {
		super(port);
	}

}
