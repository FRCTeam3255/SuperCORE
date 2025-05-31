// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frcteam3255.components.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.frcteam3255.utils.CTREModuleState;
import com.frcteam3255.utils.SN_Math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SN_SwerveModule extends SubsystemBase {

	// -*- Module-Specific -*-
	public TalonFX driveMotor;
	public TalonFX steerMotor;

	private CANcoder absoluteEncoder;
	private double absoluteEncoderOffset;

	public int moduleNumber;

	// -*- Static Motor Config -*-
	public static TalonFXConfiguration driveConfiguration;
	public static TalonFXConfiguration steerConfiguration;
	public static CANcoderConfiguration cancoderConfiguration;

	private DutyCycleOut driveMotorControllerOpen;
	private VelocityDutyCycle driveMotorControllerClosed;
	private PositionVoltage steerMotorController;

	public static NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
	public static NeutralModeValue steerNeutralMode = NeutralModeValue.Coast;
	public static InvertedValue driveInversion = InvertedValue.CounterClockwise_Positive;
	public static InvertedValue steerInversion = InvertedValue.Clockwise_Positive;
	public static SensorDirectionValue cancoderInversion = SensorDirectionValue.CounterClockwise_Positive;
	public static double minimumSteerSpeedPercent = 0.01;

	// -*- Static Physical Constants -*-
	// These default to L2s, but should be overridden
	public static double driveGearRatio = SN_SwerveConstants.MK4I.FALCON.L2.driveGearRatio;
	public static double steerGearRatio = SN_SwerveConstants.MK4I.FALCON.L2.steerGearRatio;
	public static double wheelCircumference = SN_SwerveConstants.MK4I.FALCON.L2.wheelCircumference;
	public static double maxModuleSpeedMeters = SN_SwerveConstants.MK4I.FALCON.L2.maxSpeedMeters;

	// -*- Sim -*-
	public static boolean isSimulation = false;
	private SwerveModuleState lastDesiredSwerveModuleState = new SwerveModuleState(0, new Rotation2d(0));
	private double desiredDrivePosition;
	private double timeFromLastUpdate = 0;
	private Timer simTimer = new Timer();
	private double lastSimTime = simTimer.get();

	/**
	 * This subsystem represents 1 Swerve Module. In order to use SN_SuperSwerve,
	 * you must create an array of 4 of these.
	 *
	 * @param moduleNumber
	 *            The number of the module. Typically, these are 0 to 3. 0 = Front
	 *            Left, 1 = Front Right, 2 = Back Left, 3 = Back Right
	 * @param driveMotorID
	 *            The CAN id of the drive motor
	 * @param steerMotorID
	 *            The CAN id of the steer motor
	 * @param absoluteEncoderID
	 *            The CAN id of the CANcoder
	 * @param absoluteEncoderOffset
	 *            The offset of the CANcoder in rotations. This is typically
	 *            obtained by aligning all of the wheels in the appropriate
	 *            direction and then copying the Raw Absolute encoder value.
	 * @param CANBusName
	 *            The name of the CANBus that the module is connected to.
	 */
	public SN_SwerveModule(int moduleNumber, int driveMotorID, int steerMotorID, int absoluteEncoderID,
			double absoluteEncoderOffset, String CANBusName) {

		simTimer.start();

		this.moduleNumber = moduleNumber;

		driveMotor = new TalonFX(driveMotorID, CANBusName);
		steerMotor = new TalonFX(steerMotorID, CANBusName);
		driveMotorControllerClosed = new VelocityDutyCycle(0);
		driveMotorControllerOpen = new DutyCycleOut(0);
		steerMotorController = new PositionVoltage(0);

		absoluteEncoder = new CANcoder(absoluteEncoderID, CANBusName);
		this.absoluteEncoderOffset = absoluteEncoderOffset;

		driveConfiguration = new TalonFXConfiguration();
		steerConfiguration = new TalonFXConfiguration();
		cancoderConfiguration = new CANcoderConfiguration();
	}

	public void configure() {
		// -*- Drive Motor Config -*
		driveConfiguration.MotorOutput.Inverted = driveInversion;
		driveConfiguration.MotorOutput.NeutralMode = driveNeutralMode;
		driveConfiguration.Feedback.SensorToMechanismRatio = driveGearRatio;

		driveMotor.getConfigurator().apply(driveConfiguration);

		// -*- Steer Motor Config -*-
		steerConfiguration.MotorOutput.Inverted = steerInversion;
		steerConfiguration.MotorOutput.NeutralMode = steerNeutralMode;
		steerConfiguration.Feedback.SensorToMechanismRatio = steerGearRatio;
		steerConfiguration.ClosedLoopGeneral.ContinuousWrap = true;

		steerMotor.getConfigurator().apply(steerConfiguration);

		// -*- Absolute Encoder Config -*-
		cancoderConfiguration.MagnetSensor.SensorDirection = cancoderInversion;
		absoluteEncoder.getConfigurator().apply(cancoderConfiguration);
	}

	/**
	 * Get the current raw position (no offset applied) of the module's absolute
	 * encoder. This value will NOT match the physical angle of the wheel.
	 *
	 * @return Position in rotations
	 */
	public double getRawAbsoluteEncoder() {
		// TODO: change to units using .getValue
		return absoluteEncoder.getAbsolutePosition().getValueAsDouble();
	}

	/**
	 * Get the current position, with the offset applied, of the module's absolute
	 * encoder. This value should match the physical angle of the module's wheel.
	 *
	 * @return Position in rotations, with the module's offset
	 */
	public double getAbsoluteEncoder() {
		double rotations = getRawAbsoluteEncoder();

		// "This could make the value negative but it doesn't matter." - Ian 2023
		// 99% confident that it's because it's a continuous circle
		rotations -= absoluteEncoderOffset;

		return rotations;
	}

	/**
	 * Resets the steer motor's encoder to the absolute encoder's offset value. The
	 * drive motor is not reset here because the absolute encoder does not record
	 * it's rotation.
	 */
	public void resetSteerMotorToAbsolute() {
		steerMotor.setPosition(getAbsoluteEncoder());
	}

	/**
	 * Reset the drive motor's encoder to 0.
	 */
	public void resetDriveMotorEncoder() {
		driveMotor.setPosition(0);
	}

	/**
	 * Get the current state (velocity, angle) of the module.
	 *
	 * @return Module's SwerveModuleState (velocity, angle)
	 */
	public SwerveModuleState getActualModuleState() {
		// TODO: change to units using .getValue
		double velocity = SN_Math.rotationsToMeters(driveMotor.getVelocity().getValueAsDouble(), wheelCircumference, 1);

		// TODO: change to units using .getValue
		Rotation2d angle = Rotation2d
				.fromDegrees(Units.rotationsToDegrees(steerMotor.getPosition().getValueAsDouble()));

		return new SwerveModuleState(velocity, angle);
	}

	/**
	 * Get the last desired state (velocity, angle) of the module.
	 *
	 * @return Module's Desired SwerveModuleState (velocity, angle)
	 */
	public SwerveModuleState getDesiredModuleState() {
		return lastDesiredSwerveModuleState;
	}

	/**
	 * Get the current position (distance traveled, angle) of the module. In
	 * simulation, this will return a simulated value.
	 *
	 * @return Module's SwerveModulePosition (distance, angle)
	 */
	public SwerveModulePosition getModulePosition() {
		if (isSimulation) {
			timeFromLastUpdate = simTimer.get() - lastSimTime;
			lastSimTime = simTimer.get();
			desiredDrivePosition += (lastDesiredSwerveModuleState.speedMetersPerSecond * timeFromLastUpdate);

			return new SwerveModulePosition(desiredDrivePosition, lastDesiredSwerveModuleState.angle);
		}
		// TODO: change to units using .getValue
		double distance = SN_Math.rotationsToMeters(driveMotor.getPosition().getValueAsDouble(), wheelCircumference, 1);
		// TODO: change to units using .getValue
		Rotation2d angle = Rotation2d
				.fromDegrees(Units.rotationsToDegrees(steerMotor.getPosition().getValueAsDouble()));

		return new SwerveModulePosition(distance, angle);
	}

	/**
	 * Neutral the drive motor output.
	 */
	public void neutralDriveOutput() {
		driveMotor.setControl(new NeutralOut());
	}

	/**
	 * Sets the current state (velocity and position) of the module. Given values
	 * are optimized so that the module can travel the least distance to achieve the
	 * desired value.
	 *
	 * @param desiredState
	 *            Desired velocity and angle of the module
	 * @param isOpenLoop
	 *            Is the module being set based on open loop or closed loop control
	 *
	 * @param steerInPlace
	 *            If the module can be rotated in place
	 */
	public void setModuleState(SwerveModuleState desiredState, boolean isOpenLoop, boolean steerInPlace) {
		// Optimize explanation: https://youtu.be/0Xi9yb1IMyA?t=226
		SwerveModuleState state = CTREModuleState.optimize(desiredState, getActualModuleState().angle);
		lastDesiredSwerveModuleState = state;
		// -*- Setting the Drive Motor -*-

		if (isOpenLoop) {
			// The output is from -1 to 1. Essentially a percentage
			// So, the requested speed divided by it's max speed.
			driveMotorControllerOpen.Output = (state.speedMetersPerSecond / maxModuleSpeedMeters);
			driveMotor.setControl(driveMotorControllerOpen);

		} else {
			driveMotorControllerClosed.Velocity = SN_Math.metersToRotations(state.speedMetersPerSecond,
					wheelCircumference, 1);
			driveMotor.setControl(driveMotorControllerClosed);
		}

		// -*- Setting the Steer Motor -*-

		double rotation = state.angle.getRotations();

		// If the requested speed is lower than a relevant steering speed,
		// don't turn the motor. Set it to whatever it's previous angle was.
		if (Math.abs(state.speedMetersPerSecond) < (minimumSteerSpeedPercent * maxModuleSpeedMeters) && !steerInPlace) {
			return;
		}

		steerMotorController.Position = rotation;
		steerMotor.setControl(steerMotorController);
	}
}
