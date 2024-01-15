/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.frcteam3255.utils;

public class SN_Math {
	/**
	 *
	 * @param input
	 *            - the input value used to determine the output value
	 * @param minInput
	 *            - the input value that should map to the outputAtMin value
	 * @param maxInput
	 *            - the input value that should map to the outputAtMax value
	 * @param outputAtMin
	 *            - the output value when input = minInput
	 * @param outputAtMax
	 *            - the output value when input = maxInput
	 * @return - interpolated value
	 */

	public static double interpolate(double input, double minInput, double maxInput, double outputAtMin,
			double outputAtMax) {

		double output = input;

		if (input <= minInput) {
			output = outputAtMin;
		} else if (input >= maxInput) {
			output = outputAtMax;
		} else {
			// output is somewhere between minOutput and maxOutput
			output = outputAtMin + (((input - minInput) / (maxInput - minInput)) * (outputAtMax - outputAtMin));

		}
		return output;
	}

	/**
	 * Returns signed power
	 *
	 * @param base
	 *            base number
	 * @param exponent
	 *            exponent number
	 * @return base to the power of the exponent with the original sign of the base.
	 */
	public static double signedPow(double base, double exponent) {

		double result = Math.pow(base, exponent);

		return (base > 0) ? result : -result;

	}

	public static final int TALONFX_ENCODER_PULSES_PER_COUNT = 2048;
	public static final int QUAD_ENCODER_PULSES_PER_COUNT = 4096;

	/**
	 * Converts Velocity to RPM
	 *
	 * @param a_velocity
	 *            Motor encoder counts per 100ms
	 * @param a_pulsesPerCount
	 *            Encoder counts per revolution
	 * @return Motor RPM
	 */
	public static double velocityToRPM(double a_velocity, int a_pulsesPerCount) {

		try {
			double rpm = a_velocity; // counts per 100ms
			rpm *= 10; // counts per 1s
			rpm *= 60; // counts per 1m
			rpm /= a_pulsesPerCount; // rotations per minute
			return rpm;
		} catch (ArithmeticException e) {
			System.err.println("pulsesPerCount can not be 0");
			throw e;
		}
	}

	/**
	 * Converts RPM to velocity
	 *
	 * @param a_rpm
	 *            Motor RPM
	 * @param a_pulsesPerCount
	 *            Encoder counts per revolution
	 * @return Motor encoder counts per 100ms
	 */
	public static double RPMToVelocity(double a_rpm, int a_pulsesPerCount) {
		double velocity = a_rpm; // rotations per 1m
		velocity *= a_pulsesPerCount; // counts 1m
		velocity /= 60; // counts per 1s
		velocity /= 10; // counts per 100ms
		return velocity;
	}

	// -*- Falcon v5 methods -*-

	/**
	 * Converts Falcon integrated encoder counts (falcon) to degrees From 364's
	 * BaseFalconSwerve
	 *
	 * @deprecated Only for use with Phoenix v5 devices
	 * @param counts
	 *            Falcon Integrated Encoder Counts
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism
	 * @return Degrees of Rotation of Mechanism
	 */
	public static double falconToDegrees(double counts, double gearRatio) {
		return counts * (360.0 / (gearRatio * TALONFX_ENCODER_PULSES_PER_COUNT));
	}

	/**
	 * Converts degrees to Falcon integrated encoder counts (falcon) From 364's
	 * BaseFalconSwerve
	 *
	 * @deprecated Only for use with Phoenix v5 devices
	 * @param degrees
	 *            Degrees of rotation of Mechanism
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism
	 * @return Falcon Integrated Encoder Counts
	 */
	public static double degreesToFalcon(double degrees, double gearRatio) {
		double ticks = degrees / (360.0 / (gearRatio * TALONFX_ENCODER_PULSES_PER_COUNT));
		return ticks;
	}

	/**
	 * Converts Falcon integrated encoder counts per 100 milliseconds (falcon) to
	 * RPM From 364's BaseFalconSwerve
	 *
	 * @deprecated Only for use with Phoenix v5 devices
	 * @param velocityCounts
	 *            Falcon Integrated Encoder Counts per 100 milliseconds
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
	 * @return RPM of Mechanism
	 */
	public static double falconToRPM(double velocityCounts, double gearRatio) {
		double motorRPM = velocityCounts * (600.0 / TALONFX_ENCODER_PULSES_PER_COUNT);
		double mechRPM = motorRPM / gearRatio;
		return mechRPM;
	}

	/**
	 * Converts RPM to Falcon integrated encoder counts per 100 milliseconds
	 * (falcon) From 364's BaseFalconSwerve
	 *
	 * @deprecated Only for use with Phoenix v5 devices
	 * @param RPM
	 *            RPM of mechanism
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
	 * @return Falcon Integrated Encoder Counts per 100 milliseconds
	 */
	public static double RPMToFalcon(double RPM, double gearRatio) {
		double motorRPM = RPM * gearRatio;
		double sensorCounts = motorRPM * (TALONFX_ENCODER_PULSES_PER_COUNT / 600.0);
		return sensorCounts;
	}

	/**
	 * Converts Falcon integrated encoder counts per 100 milliseconds (falcon) to
	 * Meters per Second (MPS) From 364's BaseFalconSwerve
	 *
	 * @deprecated Only for use with Phoenix v5 devices
	 * @param velocitycounts
	 *            Falcon Integrated Encoder Counts per 100 milliseconds
	 * @param circumference
	 *            Circumference of Wheel in Meters
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
	 * @return Mechanism Meters per Second
	 */
	public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
		double wheelRPM = falconToRPM(velocitycounts, gearRatio);
		double wheelMPS = (wheelRPM * circumference) / 60;
		return wheelMPS;
	}

	/**
	 * Converts Meters per Second (MPS) to Falcon integrated encoder counts per 100
	 * milliseconds (falcon) From 364's BaseFalconSwerve
	 *
	 * @deprecated Only for use with Phoenix v5 devices
	 * @param velocity
	 *            Velocity in Meters per Second
	 * @param circumference
	 *            Circumference of Wheel in Meters
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
	 * @return Falcon Integrated Encoder Counts per 100 milliseconds
	 */
	public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
		double wheelRPM = ((velocity * 60) / circumference);
		double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
		return wheelVelocity;
	}

	/**
	 * Converts Falcon integrated encoder counts to meters
	 *
	 * @deprecated Only for use with Phoenix v5 devices
	 * @param position
	 *            Falcon integrated encoder counts
	 * @param circumference
	 *            Circumference of Wheel in Meters
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism
	 * @return Meters Traveled
	 */
	public static double falconToMeters(double position, double circumference, double gearRatio) {
		double motorRotations = position / TALONFX_ENCODER_PULSES_PER_COUNT;
		double wheelRotations = motorRotations / gearRatio;
		double meters = wheelRotations * circumference;

		return meters;
	}

	/**
	 * Converts meters to Falcon integrated encoder counts
	 *
	 * @deprecated Only for use with Phoenix v5 devices
	 * @param meters
	 *            Meters Traveled
	 * @param circumference
	 *            Circumference of Wheel in Meters
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism
	 * @return Falcon integrated encoder counts
	 */
	public static double metersToFalcon(double meters, double circumference, double gearRatio) {

		double wheelRotations = meters / circumference;
		double motorRotations = wheelRotations * gearRatio;
		double encoderCounts = motorRotations * TALONFX_ENCODER_PULSES_PER_COUNT;

		return encoderCounts;
	}

	// -*- Falcon v6 Methods -*-
	// TODO: Decide which of these are actually necessary to keep since the Gear
	// Ratio is just passed into the Falcon now

	/**
	 * Converts rotations to degrees. Based off of 364's BaseFalconSwerve
	 *
	 * @param rotations
	 *            Rotations
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism
	 * @return Degrees of Rotation of Mechanism
	 */
	public static double rotationsToDegrees(double rotations, double gearRatio) {
		return rotations * (360.0 / (gearRatio));
	}

	/**
	 * Converts degrees to rotations. Based off of 364's BaseFalconSwerve
	 *
	 * @param degrees
	 *            Degrees of rotation of Mechanism
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism
	 * @return Rotations
	 */
	public static double degreesToRotations(double degrees, double gearRatio) {
		double rotations = degrees / (360.0 / (gearRatio));
		return rotations;
	}

	/**
	 * Converts Falcon rotations per second (RPM) to the mechanism's RPM. Based off
	 * of 364's BaseFalconSwerve
	 *
	 * @param velocityCounts
	 *            Falcon Rotations per second (RPM)
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism
	 * @return RPM of Mechanism
	 */
	public static double rotationsToMechanismRPM(double velocityCounts, double gearRatio) {
		double motorRPM = velocityCounts * (600.0);
		double mechRPM = motorRPM / gearRatio;
		return mechRPM;
	}

	/**
	 * Converts the mechanism's RPM to Falcon rotations per second (RPM). Based off
	 * of 364's BaseFalconSwerve
	 *
	 * @param RPM
	 *            RPM of mechanism
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism
	 * @return Falcon Rotations per second (RPM)
	 */
	public static double mechanismRPMToRotations(double RPM, double gearRatio) {
		double motorRPM = RPM * gearRatio;
		double sensorCounts = motorRPM * (600.0);
		return sensorCounts;
	}

	/**
	 * Converts Falcon rotations per second (RPM) to Meters per Second (MPS). Based
	 * off of 364's BaseFalconSwerve
	 *
	 * @param velocitycounts
	 *            Falcon Rotations per Second (RPM)
	 * @param circumference
	 *            Circumference of Wheel in Meters
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism
	 * @return Mechanism Meters per Second
	 */
	public static double rotationsToMPS(double velocitycounts, double circumference, double gearRatio) {
		double wheelRPM = rotationsToMechanismRPM(velocitycounts, gearRatio);
		double wheelMPS = (wheelRPM * circumference) / 60;
		return wheelMPS;
	}

	/**
	 * Converts Meters per Second (MPS) to Falcon rotations per second (RPM). Based
	 * off of 364's BaseFalconSwerve
	 *
	 * @param velocity
	 *            Velocity in Meters per Second
	 * @param circumference
	 *            Circumference of Wheel in Meters
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism
	 * @return Falcon Rotations per second (RPM)
	 */
	public static double MPSToFalconRotations(double velocity, double circumference, double gearRatio) {
		double wheelRPM = ((velocity * 60) / circumference);
		double wheelVelocity = mechanismRPMToRotations(wheelRPM, gearRatio);
		return wheelVelocity;
	}

	/**
	 * Converts Falcon rotations to meters
	 *
	 * @param position
	 *            Falcon rotations
	 * @param circumference
	 *            Circumference of Wheel in Meters
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism
	 * @return Meters Traveled
	 */
	public static double rotationsToMeters(double position, double circumference, double gearRatio) {
		double wheelRotations = position / gearRatio;
		double meters = wheelRotations * circumference;
		return meters;
	}

	/**
	 * Converts meters to Falcon rotations
	 *
	 * @param meters
	 *            Meters Traveled
	 * @param circumference
	 *            Circumference of Wheel in Meters
	 * @param gearRatio
	 *            Gear Ratio between Falcon and Mechanism
	 * @return Falcon rotations
	 */
	public static double metersToFalconRotations(double meters, double circumference, double gearRatio) {

		double wheelRotations = meters / circumference;
		double motorRotations = wheelRotations * gearRatio;

		return motorRotations;
	}
}
