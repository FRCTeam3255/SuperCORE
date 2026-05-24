package com.frcteam3255.components.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class SN_SuperSwerveV2 extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
	// ==========================================================
	// ****************** CTRE GENERATED CODE *******************
	// ************************ GLOBALS ************************
	// ==========================================================
	private static final double kSimLoopPeriod = 0.005; // 5 ms
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;

	/* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
	private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
	/* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
	private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
	/* Keep track if we've ever applied the operator perspective before or not */
	private boolean m_hasAppliedOperatorPerspective = false;
	/*
	 * Whether we're currently applying the X-brake request (for logging purposes)
	 */
	private boolean isCurrentlyXbraking = false;

	// ==========================================================
	// ****************** CTRE GENERATED CODE *******************
	// ********************** CONSTRUCTORS **********************
	// NOTE: Originally Extended TunerConstants.TunerSwerveDrivetrain
	// Modified to extend SwerveDrivetrain directly
	// ==========================================================
	/**
	 * Constructs a CTRE SwerveDrivetrain using the specified constants.
	 * <p>
	 * This constructs the underlying hardware devices, so users should not
	 * construct the devices themselves. If they need the devices, they can access
	 * them through getters in the classes.
	 *
	 * @param drivetrainConstants
	 *            Drivetrain-wide constants for the swerve drive
	 * @param modules
	 *            Constants for each specific module
	 */
	public SN_SuperSwerveV2(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
		super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
		if (Utils.isSimulation()) {
			startSimThread();
		}
	}

	/**
	 * Constructs a CTRE SwerveDrivetrain using the specified constants.
	 * <p>
	 * This constructs the underlying hardware devices, so users should not
	 * construct the devices themselves. If they need the devices, they can access
	 * them through getters in the classes.
	 *
	 * @param drivetrainConstants
	 *            Drivetrain-wide constants for the swerve drive
	 * @param odometryUpdateFrequency
	 *            The frequency to run the odometry loop. If unspecified or set to 0
	 *            Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
	 * @param odometryStandardDeviation
	 *            The standard deviation for odometry calculation in the form [x, y,
	 *            theta], with units in meters and radians
	 * @param visionStandardDeviation
	 *            The standard deviation for vision calculation in the form [x, y,
	 *            theta], with units in meters and radians
	 * @param modules
	 *            Constants for each specific module
	 */
	public SN_SuperSwerveV2(SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency,
			Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation,
			SwerveModuleConstants<?, ?, ?>... modules) {
		super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, odometryUpdateFrequency,
				odometryStandardDeviation, visionStandardDeviation, modules);
		if (Utils.isSimulation()) {
			startSimThread();
		}
	}

	// ==========================================================
	// ****************** CTRE GENERATED CODE *******************
	// ************************ FUNCTIONS ************************
	// ==========================================================
	/**
	 * Returns a command that applies the specified control request to this swerve
	 * drivetrain.
	 *
	 * @param requestSupplier
	 *            Function returning the request to apply
	 * @return Command to run
	 */
	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get()));
	}

	@Override
	public void periodic() {
		/*
		 * Periodically try to apply the operator perspective. If we haven't applied the
		 * operator perspective before, then we should apply it regardless of DS state.
		 * This allows us to correct the perspective in case the robot code restarts
		 * mid-match. Otherwise, only check and apply the operator perspective if the DS
		 * is disabled. This ensures driving behavior doesn't change until an explicit
		 * disable event occurs during testing.
		 */
		if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
			DriverStation.getAlliance().ifPresent(allianceColor -> {
				setOperatorPerspectiveForward(allianceColor == Alliance.Red
						? kRedAlliancePerspectiveRotation
						: kBlueAlliancePerspectiveRotation);
				m_hasAppliedOperatorPerspective = true;
			});
		}
	}

	private void startSimThread() {
		m_lastSimTime = Utils.getCurrentTimeSeconds();

		/* Run simulation at a faster rate so PID gains behave more reasonably */
		m_simNotifier = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - m_lastSimTime;
			m_lastSimTime = currentTime;

			/* use the measured time delta, get battery voltage from WPILib */
			updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		m_simNotifier.startPeriodic(kSimLoopPeriod);
	}

	/**
	 * Adds a vision measurement to the Kalman Filter. This will correct the
	 * odometry pose estimate while still accounting for measurement noise.
	 *
	 * @param visionRobotPoseMeters
	 *            The pose of the robot as measured by the vision camera.
	 * @param timestampSeconds
	 *            The timestamp of the vision measurement in seconds.
	 */
	@Override
	public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
		super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
	}

	/**
	 * Adds a vision measurement to the Kalman Filter. This will correct the
	 * odometry pose estimate while still accounting for measurement noise.
	 * <p>
	 * Note that the vision measurement standard deviations passed into this method
	 * will continue to apply to future measurements until a subsequent call to
	 * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
	 *
	 * @param visionRobotPoseMeters
	 *            The pose of the robot as measured by the vision camera.
	 * @param timestampSeconds
	 *            The timestamp of the vision measurement in seconds.
	 * @param visionMeasurementStdDevs
	 *            Standard deviations of the vision pose measurement in the form [x,
	 *            y, theta], with units in meters and radians.
	 */
	@Override
	public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
			Matrix<N3, N1> visionMeasurementStdDevs) {
		super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
				visionMeasurementStdDevs);
	}

	// ==========================================================
	// ****************** CTRE GENERATED CODE *******************
	// ************************* SYSID *************************
	// ==========================================================
	/* Swerve requests to apply during SysId characterization */
	public final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
	public final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
	public final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

	/*
	 * SysId routine for characterizing translation. This is used to find PID gains
	 * for the drive motors.
	 */
	public final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(new SysIdRoutine.Config(null, // Use default
																											// ramp rate
																											// (1 V/s)
			Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
			null, // Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
			new SysIdRoutine.Mechanism(output -> setControl(m_translationCharacterization.withVolts(output)), null,
					this));

	/*
	 * SysId routine for characterizing steer. This is used to find PID gains for
	 * the steer motors.
	 */
	public final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(new SysIdRoutine.Config(null, // Use default ramp
																									// rate (1 V/s)
			Volts.of(7), // Use dynamic voltage of 7 V
			null, // Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
			new SysIdRoutine.Mechanism(volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

	/*
	 * SysId routine for characterizing rotation. This is used to find PID gains for
	 * the FieldCentricFacingAngle HeadingController. See the documentation of
	 * SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
	 */
	public final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(new SysIdRoutine.Config(
			/* This is in radians per second, but SysId only supports "volts per second" */
			Volts.of(Math.PI / 6).per(Second),
			/* This is in radians per second, but SysId only supports "volts" */
			Volts.of(Math.PI), null, // Use default timeout (10 s)
			// Log state with SignalLogger class
			state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
			new SysIdRoutine.Mechanism(output -> {
				/* output is actually radians per second, but SysId only supports "volts" */
				setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
				/* also log the requested output for SysId */
				SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
			}, null, this));

	/* The SysId routine to test */
	public SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

	/**
	 * Runs the SysId Quasistatic test in the given direction for the routine
	 * specified by {@link #m_sysIdRoutineToApply}.
	 *
	 * @param direction
	 *            Direction of the SysId Quasistatic test
	 * @return Command to run
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysIdRoutineToApply.quasistatic(direction);
	}

	/**
	 * Runs the SysId Dynamic test in the given direction for the routine specified
	 * by {@link #m_sysIdRoutineToApply}.
	 *
	 * @param direction
	 *            Direction of the SysId Dynamic test
	 * @return Command to run
	 */
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return m_sysIdRoutineToApply.dynamic(direction);
	}

	// =========================================================
	// ********************** SUPERCORE ***********************
	// Code made by the SuperNURDs. Additions on to CTRE Swerve.
	// =========================================================
	// Swerve Requests were originally in CTRE Generated Swerve RobotContainer
	private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();
	private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest = new SwerveRequest.FieldCentricFacingAngle();

	public void drive(ChassisSpeeds chassisSpeeds) {
		setControl(fieldCentricRequest.withVelocityX(chassisSpeeds.vxMetersPerSecond)
				.withVelocityY(chassisSpeeds.vyMetersPerSecond)
				.withRotationalRate(chassisSpeeds.omegaRadiansPerSecond));
		isCurrentlyXbraking = false;
	}

	public void drive(ChassisSpeeds chassisSpeeds, Rotation2d facingAngle, double kP, double kI, double kD) {
		setControl(fieldCentricFacingAngleRequest.withVelocityX(chassisSpeeds.vxMetersPerSecond)
				.withVelocityY(chassisSpeeds.vyMetersPerSecond).withHeadingPID(kP, kI, kD)
				.withTargetDirection(facingAngle));
		isCurrentlyXbraking = false;
	}

	public void xBrake() {
		setControl(brakeRequest);
		isCurrentlyXbraking = true;
	}

	public boolean getIsCurrentlyXbraking() {
		return isCurrentlyXbraking;
	}

	/**
	 * @return The current rate of rotation for the Pigeon 2. <b>Units:</b> Degrees
	 *         per Second
	 */
	public AngularVelocity getGyroRate() {
		return this.getPigeon2().getAngularVelocityZWorld().getValue();
	}

	/**
	 * Return the current estimated pose from the pose estimator.
	 *
	 * @return The current estimated pose
	 */
	public Pose2d getPose() {
		return this.getState().Pose;
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
		double rotationVelocity = -rotationAxisSupplier.getAsDouble() * TURN_SPEED.in(Units.RadiansPerSecond);

		return new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);
	}

	/**
	 * Determines whether the robot is within the specified auto-drive zone based on
	 * the distance to a target pose.
	 *
	 * @param autoDriveMaxDistance
	 *            The maximum allowable distance for the auto-drive zone. If null,
	 *            the method will return false.
	 * @param target
	 *            The target pose to calculate the distance from the robot's current
	 *            pose.
	 * @return True if the robot's current pose is within the specified maximum
	 *         distance from the target pose, false otherwise.
	 */
	public boolean isInAutoDriveZone(Distance autoDriveMaxDistance, Pose2d target) {
		if (autoDriveMaxDistance == null) {
			return false;
		}
		Distance distanceFromPose = Units.Meters
				.of(this.getPose().getTranslation().getDistance(target.getTranslation()));
		return distanceFromPose.lt(autoDriveMaxDistance);
	}

	/**
	 * Determines whether the drivetrain's facing is within a given angular
	 * tolerance of a desired rotation.
	 *
	 * The check is performed by comparing the underlying angle measures: it returns
	 * true when the current rotation measure is strictly greater than
	 * (desiredRotation - tolerance) and strictly less than (desiredRotation +
	 * tolerance). In other words, the method tests membership in the open interval
	 * (desiredRotation - tolerance, desiredRotation + tolerance).
	 *
	 * Note: comparisons are done on the raw values returned by
	 * Rotation2d.getMeasure(). This method does not perform any additional angle
	 * wrapping or normalization beyond what Rotation2d.getMeasure() provides.
	 *
	 * @param desiredRotation
	 *            the target rotation to compare against (must not be null)
	 * @param tolerance
	 *            the allowed deviation from the target rotation (must not be null;
	 *            expected non-negative)
	 * @return true if the current rotation is strictly within the specified
	 *         tolerance of the desired rotation; false otherwise
	 * @throws NullPointerException
	 *             if desiredRotation or tolerance is null
	 */
	public boolean isAtPosition(Rotation2d desiredRotation, Angle tolerance) {
		return (this.getPose().getRotation().getMeasure().compareTo(desiredRotation.getMeasure().minus(tolerance)) > 0)
				&& this.getPose().getRotation().getMeasure()
						.compareTo(desiredRotation.getMeasure().plus(tolerance)) < 0;
	}

	/**
	 * Checks whether the robot's current pose is within a specified translational
	 * tolerance of a desired pose.
	 *
	 * <p>
	 * This method compares only the 2D translation (x, y) components of the poses
	 * and computes the Euclidean (straight-line) distance between the current pose
	 * and the desired pose. The rotational component (heading) of the poses is
	 * ignored. The comparison is inclusive: returns true when the distance is less
	 * than or equal to the provided tolerance.
	 * </p>
	 *
	 * @param desiredPose2d
	 *            the target Pose2d to compare against (must be in the same
	 *            coordinate frame as getPose())
	 * @param tolerance
	 *            a Distance representing the allowable translational error; the
	 *            method returns true if the straight-line distance to desiredPose2d
	 *            is less than or equal to this tolerance
	 * @return true if the current translation is within the given tolerance of
	 *         desiredPose2d, false otherwise
	 */
	public boolean isAtPosition(Pose2d desiredPose2d, Distance tolerance) {
		return Units.Meters.of(this.getPose().getTranslation().getDistance(desiredPose2d.getTranslation()))
				.lte(tolerance);
	}

	/**
	 * Returns whether the current angle lies strictly within the tolerance window
	 * around the target angle.
	 *
	 * The check performed is: (target - tolerance) LT current LT (target +
	 * tolerance) Comparisons are strict: equality with either boundary returns
	 * false.
	 *
	 * This method relies on Angle.minus, Angle.plus and Angle.compareTo. It does
	 * not perform any additional normalization or wrap-around handling, so callers
	 * should ensure angles are expressed in a consistent range (for example,
	 * normalized to [-180,180) or [0,360)) when necessary.
	 *
	 * @param target
	 *            the desired angle to compare against
	 * @param current
	 *            the current angle value to test
	 * @param tolerance
	 *            the allowed deviation from the target (expected to be
	 *            non-negative)
	 * @return true if current is strictly within the tolerance window around
	 *         target; false otherwise
	 */
	public boolean isAtDesiredPosition(Angle target, Angle current, Angle tolerance) {
		return (current.compareTo(target.minus(tolerance)) > 0) && current.compareTo(target.plus(tolerance)) < 0;
	}

	/**
	 * Determines whether the robot is at (or sufficiently close to) a desired pose.
	 *
	 * <p>
	 * This evaluates the Euclidean distance between the translations (x/y) of the
	 * provided target and current poses and returns true when that distance is less
	 * than or equal to the supplied tolerance.
	 *
	 * @param target
	 *            the desired Pose2d to reach (must not be null)
	 * @param current
	 *            the current Pose2d of the robot (must not be null)
	 * @param tolerance
	 *            the maximum allowable translational error as a Distance (must not
	 *            be null)
	 * @return true if the translational distance between current and target is
	 *         within tolerance, false otherwise
	 */
	public boolean isAtDesiredPosition(Pose2d target, Pose2d current, Distance tolerance) {
		Distance distance = Units.Meters.of(current.getTranslation().getDistance(target.getTranslation()));
		return distance.lte(tolerance);
	}

	/**
	 * Determines whether a two-dimensional joystick (provided as X and Y axis
	 * suppliers) is being "hit" by checking if its magnitude exceeds a given
	 * tolerance.
	 *
	 * The magnitude is computed using Math.hypot(x, y) where x and y are obtained
	 * from the provided DoubleSupplier instances.
	 *
	 * @param xAxis supplier of the joystick X-axis value; must not be null
	 * @param yAxis supplier of the joystick Y-axis value; must not be null
	 * @param tolerance non-negative threshold for considering the stick "hit";
	 *                  the method returns true when hypot(x, y) > tolerance
	 * @return true if the joystick magnitude (sqrt(x^2 + y^2)) is greater than
	 *         the specified tolerance, false otherwise
	 */
	public boolean isStickHit(DoubleSupplier xAxis, DoubleSupplier yAxis, double tolerance) {
		double rightStickX = xAxis.getAsDouble();
		double rightStickY = yAxis.getAsDouble();
		double hypotenuse = Math.hypot(rightStickX, rightStickY);

		return (hypotenuse > tolerance);
	}

	/**
	 * Determines whether a joystick/axis "hit" has occurred by comparing the absolute
	 * axis value provided by the given DoubleSupplier against a deadzone tolerance.
	 *
	 * @param axis a DoubleSupplier that returns the current axis value; must not be null
	 * @param tolerance the deadzone threshold to consider the stick as "hit" (expected non-negative)
	 * @return true if Math.abs(axis.getAsDouble()) is strictly greater than tolerance; false otherwise
	 *
	 * Example:
	 * - axis returns 0.15 and tolerance is 0.1 -> returns true
	 * - axis returns 0.1 and tolerance is 0.1 -> returns false (strict greater-than)
	 */
	public boolean isStickHit(DoubleSupplier axis, double tolerance) {
		double stickValue = Math.abs(axis.getAsDouble());
		return (stickValue > tolerance);
	}

	/**
	 * Computes the angle of the stick input in radians when the stick is "hit"
	 * (i.e. when the magnitude of the stick input is within a specified tolerance
	 * of 1). The angle is calculated using atan2 of the Y and X stick inputs, and
	 * then adjusted by subtracting the provided offset. If the stick input
	 * magnitude is outside the tolerance range, this method returns 0.
	 *
	 * @param rotationXAxis
	 *            supplier for the stick X axis
	 * @param rotationYAxis
	 *            supplier for the stick Y axis
	 * @param tolerance
	 *            allowed deviation from unit magnitude to consider the stick "hit"
	 * @param offset
	 *            an additional angle in radians to subtract from the computed stick
	 *            angle when the stick is "hit". This allows for adjusting the
	 *            reference frame or compensating for calibration offsets.
	 * @return angle in radians when stick is within tolerance, otherwise 0
	 */
	public double getStickRadians(DoubleSupplier rotationXAxis, DoubleSupplier rotationYAxis, double tolerance,
			double offset) {
		double rightStickX = rotationXAxis.getAsDouble();
		double rightStickY = rotationYAxis.getAsDouble();
		double manualDriveRotation = 0;
		if (isStickHit(rotationXAxis, rotationYAxis, tolerance)) {
			manualDriveRotation = Math.atan2(rightStickY, rightStickX) - offset;
		}
		return manualDriveRotation;
	}

	/**
	 * Computes the heading (as an {@link Angle}) from the robot's current pose to
	 * the provided target pose. The returned angle points from the robot's current
	 * position toward the target, using the field coordinate frame.
	 *
	 * The computed angle is derived from atan2(dy, dx) where dx = target.x -
	 * robot.x and dy = target.y - robot.y and converted from radians to degrees
	 * before being wrapped in a Units {@link Angle} via {@code Degrees.of(...)}.
	 *
	 * @param targetPose
	 *            goal pose to snap toward
	 * @return target heading as an {@link Angle} (degrees)
	 */
	public Angle snapToTarget(Pose2d targetPose) {
		double dx = targetPose.getX() - getPose().getX();
		double dy = targetPose.getY() - getPose().getY();
		double angleRad = Math.atan2(dy, dx);
		return Degrees.of(Math.toDegrees(angleRad));
	}

	/**
	 * Checks whether the robot is "behind" a horizontal field line defined by a
	 * distance from the blue alliance wall.
	 *
	 * For the blue alliance (isRed == false) this returns true when the robot's X
	 * position is less than {@code blueXValueInMeters}. For the red alliance (isRed
	 * == true) the field is mirrored, so the equivalent line is at
	 * {@code fieldLength - blueXValueInMeters} and we return true when the robot's
	 * X is greater than that mirrored value.
	 *
	 * Coordinates and units: this method uses the drivetrain pose's measured X
	 * coordinate (a {@link Distance}). The parameters must be supplied in the same
	 * units / measurement system (typically meters) and are compared using the
	 * Units API (e.g. {@code Distance.lt} and {@code Distance.gt}).
	 *
	 * @param blueXValueInMeters
	 *            X offset from the blue alliance wall representing the horizontal
	 *            line
	 * @param isRed
	 *            whether the robot is on the red alliance (mirror)
	 * @param fieldLength
	 *            total field length (used to compute mirrored line)
	 * @return true if the robot is behind the specified horizontal line for the
	 *         current alliance perspective
	 */
	public boolean isBehindHorizontalLine(Distance blueXValueInMeters, boolean isRed, Distance fieldLength) {
		if (!isRed) {
			boolean isDTBehindHorizontalLine = getPose().getMeasureX().lt(blueXValueInMeters);
			return isDTBehindHorizontalLine;
		} else {
			boolean isDTBehindHorizontalLine = getPose().getMeasureX().gt(fieldLength.minus(blueXValueInMeters));
			return isDTBehindHorizontalLine;
		}
	}
	/**
	 * Checks whether the robot is "behind" a vertical field line defined by a
	 * distance from the blue alliance wall along the Y axis.
	 *
	 * For the blue alliance (isRed == false) this returns true when the robot's Y
	 * position is less than {@code blueYValueInMeters}. For the red alliance (isRed
	 * == true) the equivalent line is mirrored at
	 * {@code fieldWidth - blueYValueInMeters} and we return true when the robot's Y
	 * is greater than that mirrored value.
	 *
	 * @param blueYValueInMeters
	 *            Y offset from the blue alliance wall representing the vertical
	 *            line
	 * @param isRed
	 *            whether the robot is on the red alliance
	 * @param fieldWidth
	 *            total field width (used to compute mirrored line)
	 * @return true if the robot is behind the specified vertical line for the
	 *         current alliance perspective
	 */
	public boolean isBehindVerticalLine(Distance blueYValueInMeters, boolean isRed, Distance fieldWidth) {
		if (!isRed) {
			boolean isDTBehindVerticalLine = getPose().getMeasureY().lt(blueYValueInMeters);
			return isDTBehindVerticalLine;
		} else {
			boolean isDTBehindVerticalLine = getPose().getMeasureY().gt(fieldWidth.minus(blueYValueInMeters));
			return isDTBehindVerticalLine;
		}
	}
}
