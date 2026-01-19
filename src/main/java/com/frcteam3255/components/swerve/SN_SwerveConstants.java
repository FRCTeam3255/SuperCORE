// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frcteam3255.components.swerve;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

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

	public static class ModuleLocations {
		public static final Distance frame25x25 = Inches.of(19.75).div(2);
		public static final Distance frame29x29 = Inches.of(23.75).div(2);
	}

	public static class Ratios {
		private final double steer;
		private final double drive;
		private final double couple;

		public Ratios(double steer, double drive, double couple) {
			this.steer = steer;
			this.drive = drive;
			this.couple = couple;
		}

		public double getSteer() {
			return steer;
		}

		public double getDrive() {
			return drive;
		}

		public double getCouple() {
			return couple;
		}

		public static class MK4I {
			private static final double stage1 = 1. / (14. / 50.);
			private static final double stage2L1 = 1. / (25. / 19.);
			private static final double stage2L2 = 1. / (27. / 17.);
			private static final double stage2L3 = 1. / (28. / 16.);
			private static final double stage3 = 1. / (15. / 45.);
			private static final double steer = 150. / 7.;

			private static final double driveL1 = stage1 * stage2L1 * stage3;
			private static final double driveL2 = stage1 * stage2L2 * stage3;
			private static final double driveL3 = stage1 * stage2L3 * stage3;
			private static final double couple = stage1;

			public static final Ratios L1 = new Ratios(steer, driveL1, couple);
			public static final Ratios L2 = new Ratios(steer, driveL2, couple);
			public static final Ratios L3 = new Ratios(steer, driveL3, couple);
		}
	}
}
