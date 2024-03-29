// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frcteam3255.components.swerve;

import edu.wpi.first.math.util.Units;

/**
 * <p>
 * A wrapper for a swerve module's physical constants. Contains preset constants
 * as well as a constructor for custom constants.
 * </p>
 * <p>
 * Values are taken directly from their website.
 * (https://www.swervedrivespecialties.com/products/mk4i-swerve-module)
 * </p>
 */
public class SN_SwerveConstants {
	private static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(3.8) * Math.PI;
	private static final double STEER_GEAR_RATIO = 150.0 / 7.0;
	public double steerGearRatio;
	public double wheelCircumference;
	public double driveGearRatio;
	public double maxSpeedMeters;

	/**
	 * A wrapper for Swerve Module constants. You can create your own custom
	 * constants using this constructor.
	 *
	 * @param steerGearRatio
	 *            The gear ratio between the wheel and the steer motor
	 * @param wheelCircumference
	 *            The circumference of the wheel, in meters
	 * @param driveGearRatio
	 *            The gear ratio between the wheel and the drive motor
	 * @param maxSpeedMeters
	 *            The maximum speed of the module, in meters per second
	 */
	public SN_SwerveConstants(double steerGearRatio, double wheelCircumference, double driveGearRatio,
			double maxSpeedMeters) {
		this.steerGearRatio = steerGearRatio;
		this.wheelCircumference = wheelCircumference;
		this.driveGearRatio = driveGearRatio;
		this.maxSpeedMeters = maxSpeedMeters;
	}
	public static class MK4I {
		/**
		 * Constants for modules that have a Kraken as the drive motor (the steer motor
		 * type doesn't matter here)
		 */
		public static class KRAKEN {
			/**
			 * Preset Constants for a MK4I with L1 gearing and a Kraken Drive motor
			 */
			public static final SN_SwerveConstants L1 = new SN_SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
					8.14, Units.feetToMeters(12.9));

			/**
			 * Preset Constants for a MK4I with L2 gearing and a Kraken Drive motor
			 */
			public static final SN_SwerveConstants L2 = new SN_SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
					6.75, Units.feetToMeters(15.5));

			/**
			 * Preset Constants for a MK4I with L3 gearing and a Kraken Drive motor
			 */
			public static final SN_SwerveConstants L3 = new SN_SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
					6.12, Units.feetToMeters(17.1));

			/**
			 * Preset Constants for a MK4I with L1 gearing, a Kraken Drive motor, and FOC
			 * (Field Oriented Control, a Phoenix Pro Feature) enabled
			 */
			public static final SN_SwerveConstants L1_FOC = new SN_SwerveConstants(STEER_GEAR_RATIO,
					WHEEL_CIRCUMFERENCE, 8.14, Units.feetToMeters(12.4));
			/**
			 * Preset Constants for a MK4I with L2 gearing, a Kraken Drive motor, and FOC
			 * (Field Oriented Control, a Phoenix Pro Feature) enabled
			 */
			public static final SN_SwerveConstants L2_FOC = new SN_SwerveConstants(STEER_GEAR_RATIO,
					WHEEL_CIRCUMFERENCE, 6.75, Units.feetToMeters(15.0));
			/**
			 * Preset Constants for a MK4I with L3 gearing, a Kraken Drive motor, and FOC
			 * (Field Oriented Control, a Phoenix Pro Feature) enabled
			 */
			public static final SN_SwerveConstants L3_FOC = new SN_SwerveConstants(STEER_GEAR_RATIO,
					WHEEL_CIRCUMFERENCE, 6.12, Units.feetToMeters(16.5));
		}

		/**
		 * Constants for modules that have a Falcon as the drive motor (the steer motor
		 * type doesn't matter here)
		 */
		public static class FALCON {
			/**
			 * Preset Constants for a MK4I with L1 gearing and a Falcon 500 Motor
			 */
			public static final SN_SwerveConstants L1 = new SN_SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
					8.14, Units.feetToMeters(13.7));

			/**
			 * Preset Constants for a MK4I with L2 gearing and a Falcon 500 Motor
			 */
			public static final SN_SwerveConstants L2 = new SN_SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
					6.75, Units.feetToMeters(16.5));

			/**
			 * Preset Constants for a MK4I with L3 gearing and a Falcon 500 Motor
			 */
			public static final SN_SwerveConstants L3 = new SN_SwerveConstants(STEER_GEAR_RATIO, WHEEL_CIRCUMFERENCE,
					6.12, Units.feetToMeters(18.2));

			/**
			 * Preset Constants for a MK4I with L1 gearing, a Falcon 500 Motor, and FOC
			 * (Field Oriented Control, a Phoenix Pro Feature) enabled
			 */
			public static final SN_SwerveConstants L1_FOC = new SN_SwerveConstants(STEER_GEAR_RATIO,
					WHEEL_CIRCUMFERENCE, 8.14, Units.feetToMeters(13.0));
			/**
			 * Preset Constants for a MK4I with L2 gearing, a Falcon 500 Motor, and FOC
			 * (Field Oriented Control, a Phoenix Pro Feature) enabled
			 */
			public static final SN_SwerveConstants L2_FOC = new SN_SwerveConstants(STEER_GEAR_RATIO,
					WHEEL_CIRCUMFERENCE, 6.75, Units.feetToMeters(15.7));
			/**
			 * Preset Constants for a MK4I with L3 gearing, a Falcon 500 Motor, and FOC
			 * (Field Oriented Control, a Phoenix Pro Feature) enabled
			 */
			public static final SN_SwerveConstants L3_FOC = new SN_SwerveConstants(STEER_GEAR_RATIO,
					WHEEL_CIRCUMFERENCE, 6.12, Units.feetToMeters(17.3));
		}
	}
}
