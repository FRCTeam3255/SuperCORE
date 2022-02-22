package com.frcteam3255.utils;

public class SN_RPM {

	private static final double TALONFX_COUNTS_PER_ROTATION = 2048;

	/**
	 * Converts TalonFX Velocity to RPM
	 *
	 * @param a_velocity
	 *            Motor encoder counts per 100ms
	 * @return Motor RPM
	 */
	public static double talonFXVelocityToRPM(double a_velocity) {
		double rpm = a_velocity; // counts per 100ms
		rpm *= 10; // counts per 1s
		rpm *= 60; // counts per 1m
		rpm /= TALONFX_COUNTS_PER_ROTATION; // rotations per minute
		return rpm;
	}

	/**
	 * Converts RPM to TalonFX velocity
	 *
	 * @param a_rpm
	 *            Motor RPM
	 * @return Motor encoder counts per 100ms
	 */
	public static double talonFXRPMToVelocity(double a_rpm) {
		double velocity = a_rpm; // rotations per 1m
		velocity *= TALONFX_COUNTS_PER_ROTATION; // counts 1m
		velocity /= 60; // counts per 1s
		velocity /= 10; // counts per 100ms
		return velocity;
	}
}
