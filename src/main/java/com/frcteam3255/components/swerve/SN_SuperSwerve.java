// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frcteam3255.components.swerve;

import java.util.HashMap;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
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
	/**
	 * Drive base radius in meters. Distance from robot center to furthest module.
	 */
	private double driveBaseRadius;
	public PIDConstants autoDrivePID;
	public PIDConstants autoSteerPID;
	private Matrix<N3, N1> stateStdDevs;
	private Matrix<N3, N1> visionStdDevs;
	public HashMap<String, Command> autoEventMap = new HashMap<>();
	public ReplanningConfig autoReplanningConfig;
	public boolean autoFlipPaths;

	public PathPlannerTrajectory exampleAuto;

	private boolean isSimulation;
	public double simAngle = 0;
	private SwerveModuleState[] lastDesiredStates = new SwerveModuleState[]{new SwerveModuleState(),
			new SwerveModuleState(), new SwerveModuleState(), new SwerveModuleState()};
	public double timeFromLastUpdate = 0;
	public double lastSimTime = Timer.getFPGATimestamp();
	public Field2d field;

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
	 *            The name of the CANBus that all of the swerve components are on
	 * @param pigeonCANId
	 *            The CAN id of the Pigeon. The Pigeon MUST be on the same CANBus as
	 *            the modules
	 * @param minimumSteerPercent
	 *            The minimum PercentOutput required to make the steer motor move
	 * @param driveInversion
	 *            The direction that is positive for drive motors
	 * @param steerInversion
	 *            The direction that is positive for steer motors
	 * @param cancoderInversion
	 *            The direction that is positive for Cancoders
	 * @param driveNeutralMode
	 *            The behavior of every drive motor when set to neutral-output
	 * @param steerNeutralMode
	 *            The behavior of every steer motor when set to neutral-output
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
	 * @param autoReplanningConfig
	 *            The configuration for replanning paths in autonomous. See the
	 *            <a href=
	 *            "https://mjansen4857.com/pathplanner/docs/java/com/pathplanner/lib/util/ReplanningConfig.html">PathPlanner
	 *            API</a> for more information
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
			InvertedValue driveInversion, InvertedValue steerInversion, SensorDirectionValue cancoderInversion,
			NeutralModeValue driveNeutralMode, NeutralModeValue steerNeutralMode, Matrix<N3, N1> stateStdDevs,
			Matrix<N3, N1> visionStdDevs, PIDConstants autoDrivePID, PIDConstants autoSteerPID,
			ReplanningConfig autoReplanningConfig, boolean autoFlipPaths, boolean isSimulation) {

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
		this.autoDrivePID = autoDrivePID;
		this.autoSteerPID = autoSteerPID;
		this.isSimulation = isSimulation;
		this.autoReplanningConfig = autoReplanningConfig;
		this.autoFlipPaths = autoFlipPaths;

		SN_SwerveModule.isSimulation = isSimulation;
		SN_SwerveModule.wheelCircumference = swerveConstants.wheelCircumference;
		SN_SwerveModule.maxModuleSpeedMeters = swerveConstants.maxSpeedMeters;
		SN_SwerveModule.driveGearRatio = swerveConstants.driveGearRatio;
		SN_SwerveModule.steerGearRatio = swerveConstants.steerGearRatio;

		SN_SwerveModule.CANBusName = CANBusName;
		SN_SwerveModule.minimumSteerSpeedPercent = minimumSteerPercent;

		SN_SwerveModule.driveInversion = driveInversion;
		SN_SwerveModule.driveNeutralMode = driveNeutralMode;

		SN_SwerveModule.steerInversion = steerInversion;
		SN_SwerveModule.steerNeutralMode = steerNeutralMode;
		SN_SwerveModule.cancoderInversion = cancoderInversion;

		driveBaseRadius = Math.sqrt(Math.pow((wheelbase / 2), 2) + Math.pow((trackWidth / 2), 2));
		pigeon = new Pigeon2(pigeonCANId, CANBusName);

		// The absolute encoders need time to initialize
		Timer.delay(2.5);
		resetModulesToAbsolute();
		configure();

		AutoBuilder.configureHolonomic(this::getPose, this::resetPoseToPose, this::getChassisSpeeds,
				this::driveAutonomous, new HolonomicPathFollowerConfig(autoDrivePID, autoSteerPID,
						swerveConstants.maxSpeedMeters, driveBaseRadius, autoReplanningConfig),
				() -> autoFlipPaths, this);
	}

	public void configure() {
		for (SN_SwerveModule mod : modules) {
			mod.configure();
		}

		swervePoseEstimator = new SwerveDrivePoseEstimator(swerveKinematics, getRotation(), getModulePositions(),
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
			states[mod.moduleNumber] = mod.getModuleState();
		}

		return states;
	}

	/**
	 * Get the last desired states (velocity, angle) of each module.
	 *
	 * @return An Array of Swerve module states (velocity, angle)
	 */
	public SwerveModuleState[] getDesiredModuleStates() {
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
			mod.setModuleState(desiredModuleStates[mod.moduleNumber], isOpenLoop);
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
	 * Drive the drivetrain in autonomous. Autonomous driving is always closed loop.
	 *
	 * @param chassisSpeeds
	 *            Desired robot-relative chassis speeds
	 *
	 */
	public void driveAutonomous(ChassisSpeeds chassisSpeeds) {
		SwerveModuleState[] desiredModuleStates = swerveKinematics
				.toSwerveModuleStates(ChassisSpeeds.discretize(chassisSpeeds, timeFromLastUpdate));
		setModuleStates(desiredModuleStates, true);
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
		swervePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), getRotation(), getModulePositions());
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
	 * Reset the pose estimator's pose to a given pose.
	 *
	 * @param pose
	 *            The pose you would like to reset the pose estimator to
	 */
	public void resetPoseToPose(Pose2d pose) {
		swervePoseEstimator.resetPosition(getRotation(), getModulePositions(), pose);
	}

	/**
	 * Get the rotation of the drivetrain using the Pigeon. In simulation, this will
	 * return a simulated value.
	 *
	 * @return Rotation of drivetrain. The Rotation2d object will deal with units
	 *         for you as long as you specify your desired unit (ex.
	 *         rotation.getDegrees)
	 */
	public Rotation2d getRotation() {
		if (isSimulation && lastDesiredStates != null) {
			simAngle += swerveKinematics.toChassisSpeeds(lastDesiredStates).omegaRadiansPerSecond * timeFromLastUpdate;
			return Rotation2d.fromRadians(simAngle);
		}
		return Rotation2d.fromDegrees(pigeon.getYaw().getValue());
	}

	/**
	 * Resets the Yaw of the Pigeon to 0
	 */
	public void resetYaw() {
		pigeon.setYaw(0);
	}

	/**
	 * Resets the Yaw of the Pigeon to the given value
	 *
	 * @param yaw
	 *            The yaw (in degrees) to reset the Pigeon to
	 */
	public void resetYaw(double yaw) {
		pigeon.setYaw(yaw);
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

	@Override
	public void periodic() {
		updateTimer();
		updatePoseEstimator();
	}
}
