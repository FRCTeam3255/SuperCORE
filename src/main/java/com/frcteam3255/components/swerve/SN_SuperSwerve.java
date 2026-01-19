// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frcteam3255.components.swerve;

import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SN_SuperSwerve extends SubsystemBase {
	public SN_SwerveModule[] modules;
	public SwerveDrivePoseEstimator swervePoseEstimator;
	public SwerveDriveKinematics swerveKinematics;
	public Pigeon2 pigeon;
	public boolean isFieldRelative;

	public SN_SwerveConstants swerveConstants;
	private Matrix<N3, N1> stateStdDevs;
	private Matrix<N3, N1> visionStdDevs;
	public HashMap<String, Command> autoEventMap = new HashMap<>();
	public BooleanSupplier autoFlipPaths;

	private boolean isSimulation;
	public double simAngle = 0;
	private SwerveModuleState[] lastDesiredStates = new SwerveModuleState[]{new SwerveModuleState(),
			new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
	public double timeFromLastUpdate = 0;
	public double lastSimTime = Timer.getFPGATimestamp();
	public Field2d field;
	public TalonFXConfiguration driveConfig, steerConfig;
	public CANcoderConfiguration cancoderConfig;
	public HolonomicDriveController teleopAutoDriveController;
	public AngularVelocity turnSpeed;

	/**
	 * <p>
	 * A superclass for a Swerve Drive. Your Drivetrain subsystem should extend from
	 * this class. In addition to the parameters required, it is highly recommended
	 * that you override SN_SwerveModule's driveConfiguration, steerConfiguration,
	 * and driveFeedForward in your configure method.
	 * </p>
	 * <p>
	 * View an example implementation here:
	 * https://github.com/FRCTeam3255/Standard_Swerve_Code
	 * </p>
	 *
	 * @param swerveConstants
	 *            The constants for all of your modules, such as gear ratio and max
	 *            speed. You can create your own SN_SwerveConstants object or use a
	 *            preset.
	 * @param modules
	 *            An array of SN_SwerveModules.
	 * @param wheelbase
	 *            Physically measured distance (center to center) between the Left
	 *            and Right wheels in meters
	 * @param trackWidth
	 *            Physically measured distance (center to center) between the Front
	 *            and Back wheels in meters
	 * @param CANBusName
	 *            The name of the CANBus that the Pigeon is on.
	 * @param pigeonCANId
	 *            The CAN id of the Pigeon.
	 * @param minimumSteerPercent
	 *            The minimum PercentOutput required to make the steer motor move
	 * @param driveConfig
	 *            The configuration for each drive motor. Make sure you set the
	 *            inversion, neutral mode, and sensor to mechanism ratio!
	 * @param steerConfig
	 *            The configuration for each steer motor. Make sure you set the
	 *            inversion, neutral mode, sensor to mechanism ratio, and continuous
	 *            wrap!
	 * @param cancoderConfig
	 *            The configuration for each CANCoder. Make sure you set the sensor
	 *            direction!
	 * @param stateStdDevs
	 *            Standard deviations of the pose estimate (x position in meters, y
	 *            position in meters, and heading in radians). Increase these
	 *            numbers to trust your state estimate less.
	 * @param visionStdDevs
	 *            Standard deviations of vision pose measurements (x position in
	 *            meters, y position in meters, and heading in radians). Increase
	 *            these numbers to trust the vision pose measurement less.
	 * @param autoDrivePID
	 *            The translational PID constants applied to the entire Drivetrain
	 *            during autonomous in order to reach the correct pose
	 * @param autoSteerPID
	 *            The rotational PID constants applied to the entire Drivetrain
	 *            during autonomous in order to reach the correct pose
	 * @param teleopAutoDriveController
	 *            The PID controller used to control rotational snapping and auto
	 *            driving during Teleoperated.
	 * @param turnSpeed
	 *            The turning speed of the robot.
	 * @param robotConfig
	 *            The robot configuration used by PathPlanner.
	 * @param autoFlipPaths
	 *            Determines if paths should be flipped to the other side of the
	 *            field. This will maintain a global blue alliance origin. Used for
	 *            PathPlanner
	 * @param isSimulation
	 *            If your robot is running in Simulation. As of 2023, you can supply
	 *            this with Robot.isSimulation();
	 */
	public SN_SuperSwerve(SN_SwerveConstants swerveConstants, SN_SwerveModule[] modules, double wheelbase,
			double trackWidth, String CANBusName, int pigeonCANId, double minimumSteerPercent,
			TalonFXConfiguration driveConfig, TalonFXConfiguration steerConfig, CANcoderConfiguration cancoderConfig,
			Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionStdDevs,
			HolonomicDriveController teleopAutoDriveController, AngularVelocity turnSpeed,
			BooleanSupplier autoFlipPaths, boolean isSimulation) {

		isFieldRelative = true;
		field = new Field2d();

		// Location of all modules in the WPILib robot coordinate system
		swerveKinematics = new SwerveDriveKinematics(new Translation2d(wheelbase / 2.0, trackWidth / 2.0),
				new Translation2d(wheelbase / 2.0, -trackWidth / 2.0),
				new Translation2d(-wheelbase / 2.0, trackWidth / 2.0),
				new Translation2d(-wheelbase / 2.0, -trackWidth / 2.0));

		this.modules = modules;
		this.stateStdDevs = stateStdDevs;
		this.visionStdDevs = visionStdDevs;
		this.swerveConstants = swerveConstants;
		this.isSimulation = isSimulation;
		this.autoFlipPaths = autoFlipPaths;
		this.teleopAutoDriveController = teleopAutoDriveController;
		this.turnSpeed = turnSpeed;
		this.driveConfig = driveConfig;
		this.steerConfig = steerConfig;
		this.cancoderConfig = cancoderConfig;

		SN_SwerveModule.isSimulation = isSimulation;
		SN_SwerveModule.wheelCircumference = swerveConstants.wheelCircumference;
		SN_SwerveModule.maxModuleSpeedMeters = swerveConstants.maxSpeedMeters;

		SN_SwerveModule.minimumSteerSpeedPercent = minimumSteerPercent;

		pigeon = new Pigeon2(pigeonCANId, CANBusName);

		// The absolute encoders need time to initialize
		Timer.delay(2.5);
		resetModulesToAbsolute();
		configure();
	}

	public void configure() {
		SN_SwerveModule.driveConfiguration = driveConfig;
		SN_SwerveModule.steerConfiguration = steerConfig;
		SN_SwerveModule.cancoderConfiguration = cancoderConfig;
		for (SN_SwerveModule mod : modules) {
			mod.configure();
		}

		swervePoseEstimator = new SwerveDrivePoseEstimator(swerveKinematics, getGyroRotation(), getModulePositions(),
				new Pose2d(), stateStdDevs, visionStdDevs);

	}

	/**
	 * Reset all of the steer motors's encoders to the absolute encoder values.
	 */
	public void resetModulesToAbsolute() {
		for (SN_SwerveModule mod : modules) {
			mod.resetSteerMotorToAbsolute();
		}
	}

	/**
	 * Get the position (distance, angle) of each module.
	 *
	 * @return An Array of Swerve module positions (distance, angle)
	 */
	public SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] positions = new SwerveModulePosition[4];

		for (SN_SwerveModule mod : modules) {
			positions[mod.moduleNumber] = mod.getModulePosition();
		}

		return positions;
	}

	/**
	 * Get the actual state (velocity, angle) of each module.
	 *
	 * @return An Array of Swerve module states (velocity, angle)
	 */
	public SwerveModuleState[] getActualModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];

		for (SN_SwerveModule mod : modules) {
			states[mod.moduleNumber] = mod.getActualModuleState();
		}

		return states;
	}

	/**
	 * Get the last desired states (velocity, angle) of each module.
	 *
	 * @return An Array of Swerve module states (velocity, angle)
	 */
	public SwerveModuleState[] getDesiredModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];

		for (SN_SwerveModule mod : modules) {
			states[mod.moduleNumber] = mod.getDesiredModuleState();
		}

		return lastDesiredStates;
	}

	/**
	 * Returns the robot-relative chassis speeds.
	 *
	 * @return The robot-relative chassis speeds
	 */
	public ChassisSpeeds getChassisSpeeds() {
		return swerveKinematics.toChassisSpeeds(getActualModuleStates());
	}

	/**
	 * Set the states (velocity and position) of the modules.
	 *
	 * @param desiredModuleStates
	 *            Desired states to set the modules to
	 * @param isOpenLoop
	 *            Are the modules being set based on open loop or closed loop
	 *            control
	 *
	 */
	public void setModuleStates(SwerveModuleState[] desiredModuleStates, boolean isOpenLoop) {
		// Lowers the speeds if needed so that they are actually achievable. This has to
		// be done here because speeds must be lowered relative to the other speeds as
		// well
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleStates, swerveConstants.maxSpeedMeters);
		lastDesiredStates = desiredModuleStates;

		for (SN_SwerveModule mod : modules) {
			mod.setModuleState(desiredModuleStates[mod.moduleNumber], isOpenLoop, false);
		}
	}

	/**
	 * Drive the drivetrain!
	 *
	 * @param translation
	 *            Desired translational velocity in meters per second
	 * @param rotation
	 *            Desired rotational velocity in radians per second
	 * @param isOpenLoop
	 *            Are the modules being set based on open loop or closed loop
	 *            control
	 *
	 */
	public void drive(Translation2d translation, double rotation, boolean isOpenLoop) {
		ChassisSpeeds chassisSpeeds;

		if (isFieldRelative) {
			chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
					getRotation());
		} else {
			chassisSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
		}

		SwerveModuleState[] desiredModuleStates = swerveKinematics
				.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, timeFromLastUpdate));
		setModuleStates(desiredModuleStates, isOpenLoop);
	}

	/**
	 * Drive the drivetrain with pre-calculated ChassisSpeeds
	 *
	 * @param chassisSpeeds
	 *            Desired ChassisSpeeds
	 * @param isOpenLoop
	 *            Are the modules being set based on open loop or closed loop
	 *            control
	 *
	 */
	public void drive(ChassisSpeeds chassisSpeeds, boolean isOpenLoop) {
		SwerveModuleState[] desiredModuleStates = swerveKinematics
				.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, timeFromLastUpdate));
		setModuleStates(desiredModuleStates, isOpenLoop);
	}

	/**
	 * Drive the drivetrain in autonomous. Autonomous driving is always closed loop.
	 *
	 * @param chassisSpeeds
	 *            Desired robot-relative chassis speeds
	 *
	 */
	public void driveAutonomous(ChassisSpeeds chassisSpeeds) {
		drive(chassisSpeeds, false);
	}

	/**
	 * Sets all modules to neutral output
	 */
	public void neutralDriveOutputs() {
		for (SN_SwerveModule mod : modules) {
			mod.neutralDriveOutput();
		}
	}

	/**
	 * Sets the drive method to use field relative drive controls
	 */
	public void setFieldRelative() {
		isFieldRelative = true;
	}

	/**
	 * Sets the drive method to use robot relative drive controls
	 */
	public void setRobotRelative() {
		isFieldRelative = false;
	}

	/**
	 * Updates the pose estimator with the current robot uptime, the gyro yaw, and
	 * each swerve module position.
	 * <p>
	 * This method MUST be called every loop (or else pose estimator breaks)
	 */
	public void updatePoseEstimator() {
		swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getGyroRotation(), getModulePositions());
	}

	/**
	 * Return the current estimated pose from the pose estimator.
	 *
	 * @return The current estimated pose
	 */
	public Pose2d getPose() {
		return swervePoseEstimator.getEstimatedPosition();
	}

	/**
	 * Return the current rotation of the robot using the pose estimator.
	 *
	 * @return The current estimated rotation
	 */
	public Rotation2d getRotation() {
		return swervePoseEstimator.getEstimatedPosition().getRotation();
	}

	/**
	 * Reset the pose estimator's pose to a given pose.
	 *
	 * @param pose
	 *            The pose you would like to reset the pose estimator to
	 */
	public void resetPoseToPose(Pose2d pose) {
		swervePoseEstimator.resetPosition(getGyroRotation(), getModulePositions(), pose);
	}

	/**
	 * Get the rotation of the drivetrain using the Pigeon. In simulation, this will
	 * return a simulated value. The rotation will wrap from 0 to 360 degrees.
	 *
	 * @return Rotation of drivetrain. The Rotation2d object will deal with units
	 *         for you as long as you specify your desired unit (ex.
	 *         rotation.getDegrees)
	 */
	private Rotation2d getGyroRotation() {
		if (isSimulation && lastDesiredStates != null) {
			simAngle += swerveKinematics.toChassisSpeeds(lastDesiredStates).omegaRadiansPerSecond * timeFromLastUpdate;

			// Wrap to +- 1 rotation
			simAngle = simAngle % (2 * Math.PI);
			// Wrap to 0 -> 1 rotation
			simAngle = (simAngle < 0) ? simAngle + (2 * Math.PI) : simAngle;
			return Rotation2d.fromRadians(simAngle);
		}
		double yaw = pigeon.getYaw().getValueAsDouble() % 360;
		return (yaw < 0) ? Rotation2d.fromDegrees(yaw + 360) : Rotation2d.fromDegrees(yaw);
	}

	/**
	 * @return The current rate of rotation for the Pigeon 2. <b>Units:</b> Degrees
	 *         per Second
	 */
	public double getGyroRate() {
		return pigeon.getAngularVelocityZWorld().getValueAsDouble();
	}

	/**
	 * Resets the Yaw of the Pigeon to the given value
	 *
	 * @param yaw
	 *            The yaw (in degrees) to reset the Pigeon to
	 */
	private void resetYaw(double yaw) {
		if (isSimulation) {
			simAngle = Units.Radians.convertFrom(yaw, Units.Degrees);
		}

		pigeon.setYaw(yaw);
	}

	/**
	 * Adds a vision measurement to the pose estimator. This method does not need to
	 * be called periodically. The superclass of this class (SN_SuperSwerve) already
	 * updates the pose estimator every loop.
	 *
	 * @param estimatedPose
	 *            The estimated pose measurement generated by vision
	 * @param timestamp
	 *            The timestamp of that pose estimate (not necessarily the current
	 *            timestamp)
	 */
	public void addVisionMeasurement(Pose2d estimatedPose, double timestamp) {
		swervePoseEstimator.addVisionMeasurement(estimatedPose, timestamp);
	}

	/**
	 * <p>
	 * <b>Must be run periodically in order to function properly!</b>
	 * </p>
	 * Updates the values based on the current timestamp of the robot. Mainly
	 * required for simulation and discretize
	 */
	public void updateTimer() {
		timeFromLastUpdate = Timer.getFPGATimestamp() - lastSimTime;
		lastSimTime = Timer.getFPGATimestamp();
	}

	/**
	 * Returns the rotational velocity calculated with PID control to reach the
	 * given rotation. This must be called every loop until you reach the given
	 * rotation.
	 *
	 * @param desiredYaw
	 *            The desired yaw to rotate to
	 * @return The desired velocity needed to rotate to that position.
	 */
	public AngularVelocity getVelocityToRotate(Rotation2d desiredYaw) {
		double yawSetpoint = teleopAutoDriveController.getThetaController().calculate(getRotation().getRadians(),
				desiredYaw.getRadians());

		// limit the PID output to our maximum rotational speed
		yawSetpoint = MathUtil.clamp(yawSetpoint, -turnSpeed.in(Units.RadiansPerSecond),
				turnSpeed.in(Units.RadiansPerSecond));

		return Units.RadiansPerSecond.of(yawSetpoint);
	}

	// I would write "field relative" or "robot relative" somewhere here but I
	// genuinely don't know if its robot relative (i think it is?)

	/**
	 * Aligns the robot's rotation to a desired target while maintaining the given
	 * translational velocities. This method is typically used for rotational-only
	 * auto-alignment during swerve drive operations.
	 *
	 * @param desiredTarget
	 *            The desired pose to align the robot's rotation to. Only the
	 *            rotation component of the pose is used.
	 * @param velocities
	 *            The translational velocities (vx and vy) to maintain while
	 *            aligning the rotation.
	 * @param isOpenLoop
	 *            If true, the drive system operates in open-loop mode; otherwise,
	 *            it operates in closed-loop mode.
	 */

	public void rotationalAlign(Pose2d desiredTarget, ChassisSpeeds velocities, boolean isOpenLoop) {
		// Rotational-only auto-align
		drive(new Translation2d(velocities.vxMetersPerSecond, velocities.vyMetersPerSecond),
				getVelocityToRotate(desiredTarget.getRotation()).in(Units.RadiansPerSecond), isOpenLoop);
	}

	/**
	 * Automatically aligns the robot to a desired target pose while allowing for
	 * manual adjustments and additional control options.
	 *
	 * @param desiredTarget
	 *            The target pose (position and orientation) to align to.
	 * @param manualVelocities
	 *            The manually controlled velocities to be applied, used when
	 *            locking X or Y axes.
	 * @param isOpenLoop
	 *            Whether the drive system should operate in open-loop mode.
	 * @param lockX
	 *            If true, the X-axis velocity will be locked to the manual
	 *            velocity.
	 * @param lockY
	 *            If true, the Y-axis velocity will be locked to the manual
	 *            velocity.
	 * @param invertRotation
	 *            If true, the rotational velocity will be inverted.
	 */
	public void autoAlign(Pose2d desiredTarget, ChassisSpeeds manualVelocities, boolean isOpenLoop, boolean lockX,
			boolean lockY, boolean invertRotation) {

		// Full auto-align
		ChassisSpeeds automatedDTVelocity = teleopAutoDriveController.calculate(getPose(), desiredTarget, 0,
				desiredTarget.getRotation());

		if (lockX) {
			automatedDTVelocity.vxMetersPerSecond = manualVelocities.vxMetersPerSecond;
		}
		if (lockY) {
			automatedDTVelocity.vyMetersPerSecond = manualVelocities.vyMetersPerSecond;
		}
		if (invertRotation) {
			automatedDTVelocity.omegaRadiansPerSecond = -automatedDTVelocity.omegaRadiansPerSecond;
		}
		drive(automatedDTVelocity, isOpenLoop);
	}

	public boolean isAtRotation(Rotation2d desiredRotation, Angle tolerance) {
		return (getRotation().getMeasure().compareTo(desiredRotation.getMeasure().minus(tolerance)) > 0)
				&& getRotation().getMeasure().compareTo(desiredRotation.getMeasure().plus(tolerance)) < 0;
	}

	public boolean isAtPosition(Pose2d desiredPose2d, Distance tolerance) {
		return Units.Meters.of(getPose().getTranslation().getDistance(desiredPose2d.getTranslation())).lte(tolerance);
	}
	/**
	 * Calculate which pose from an array has the closest rotation to the robot's
	 * current pose. If multiple poses have the same rotation, the last one in the
	 * list will be returned.
	 *
	 * @param poses
	 *            The list of poses to check
	 * @return The last pose in the list with the closest rotation
	 */
	public Pose2d getClosestPoseByRotation(List<Pose2d> poses) {
		Pose2d closestPose = poses.get(0);
		double smallestDifference = Math.abs(getRotation().minus(closestPose.getRotation()).getRadians());
		for (Pose2d pose : poses) {
			double difference = Math.abs(getRotation().minus(pose.getRotation()).getRadians());
			if (difference < smallestDifference) {
				smallestDifference = difference;
				closestPose = pose;
			}
		}
		return closestPose;
	}

	/**
	 * Calculates the chassis velocities based on joystick inputs and other
	 * parameters.
	 *
	 * @param xAxisSupplier
	 *            A DoubleSupplier providing the x-axis input for forward/backward
	 *            movement.
	 * @param yAxisSupplier
	 *            A DoubleSupplier providing the y-axis input for left/right
	 *            movement.
	 * @param rotationAxisSupplier
	 *            A DoubleSupplier providing the rotation input for turning.
	 * @param slowMode
	 *            A BooleanSupplier indicating whether the slow mode is active.
	 * @param isRed
	 *            A boolean indicating if the robot is on the red alliance (reverses
	 *            controls if true).
	 * @param SLOW_MODE_MULTIPLIER
	 *            A multiplier applied to velocities when slow mode is active.
	 * @param REAL_DRIVE_SPEED
	 *            The maximum linear velocity of the robot in meters per second.
	 * @param TURN_SPEED
	 *            The maximum angular velocity of the robot in radians per second.
	 * @return A ChassisSpeeds object containing the calculated x, y, and rotational
	 *         velocities.
	 */
	public ChassisSpeeds calculateVelocitiesFromInput(DoubleSupplier xAxisSupplier, DoubleSupplier yAxisSupplier,
			DoubleSupplier rotationAxisSupplier, BooleanSupplier slowMode, boolean isRed, double SLOW_MODE_MULTIPLIER,
			LinearVelocity REAL_DRIVE_SPEED, AngularVelocity TURN_SPEED) {

		double redAllianceMultiplier = isRed ? -1 : 1;
		double slowModeMultiplier = slowMode.getAsBoolean() ? SLOW_MODE_MULTIPLIER : 1.0;

		double xVelocity = xAxisSupplier.getAsDouble() * REAL_DRIVE_SPEED.in(Units.MetersPerSecond)
				* redAllianceMultiplier * slowModeMultiplier;
		double yVelocity = -yAxisSupplier.getAsDouble() * REAL_DRIVE_SPEED.in(Units.MetersPerSecond)
				* redAllianceMultiplier * slowModeMultiplier;
		double rotationVelocity = rotationAxisSupplier.getAsDouble() * TURN_SPEED.in(Units.RadiansPerSecond);

		return new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);
	}

	@Override
	public void periodic() {
		updateTimer();
		updatePoseEstimator();
	}
}
