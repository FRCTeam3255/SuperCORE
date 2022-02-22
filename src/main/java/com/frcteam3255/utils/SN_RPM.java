package com.frcteam3255.utils;

public class SN_RPM {

	public static final double TALONFX_ENCODER_PULSES_PER_COUNT = 2048;
	public static final double QUAD_ENCODER_PULSES_PER_COUNT = 4096;

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
			return 0;
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
