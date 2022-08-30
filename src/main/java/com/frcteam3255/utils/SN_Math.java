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
}
