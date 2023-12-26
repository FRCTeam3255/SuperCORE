package com.frcteam3255.utils;

/**
 * Linear Interpolation Table for finding the best value based on limited data
 * points. "Borrowed" from 1706's 2022 code.
 * <p>
 * How to use:
 * <p>
 * 1. Make list of SN_Point2D's that contain gathered information
 * <p>
 * 2. Create new SN_Lerp with this list of points
 * <p>
 * 3. On the created SN_Lerp, call {@link #getOutput()} with the desired x value
 * and it will return a linearly interpolated y value between the two closest
 * datapoints
 */
public class SN_Lerp {
	private double m_maxInput = Double.NEGATIVE_INFINITY;
	private double m_minInput = Double.POSITIVE_INFINITY;
	private final SN_Point2D[] m_points;
	public final int size;

	/**
	 * Create new SN_Lerp
	 *
	 * @param points
	 *            List of SN_Point2D's
	 */
	public SN_Lerp(SN_Point2D... points) {
		m_points = points;
		size = m_points.length;
		for (int i = 0; i < size; i++) {
			if (m_points[i].getX() > m_maxInput) {
				m_maxInput = m_points[i].getX();
			}
			if (m_points[i].getX() < m_minInput) {
				m_minInput = m_points[i].getX();
			}
		}

	}

	/**
	 * Get a linearly interpolated output from the table
	 * <p>
	 * Sequentially searches through all values of table. May not perform well with
	 * large datasets
	 *
	 * @param input
	 *            X input
	 * @return Linearly interpolated Y value between the two closest datapoints
	 */
	public double getOutput(double input) {
		int index = 0;
		if (input <= m_minInput) {
			index = 0;
		} else if (input >= m_maxInput) {
			index = size - 2;
		} else {
			for (int i = 1; i < m_points.length; i++) {
				if (input > m_points[i - 1].getX() && input <= m_points[i].getX()) {
					index = i - 1;
				}
			}
		}
		return interpolate(input, m_points[index], m_points[index + 1]);
	}

	/**
	 * Interpolate between two SN_Point2Ds
	 *
	 * @param input
	 *            Desired X value
	 * @param point1
	 *            Closest point below desired X value
	 * @param point2
	 *            Closest point above desired X value
	 * @return interpolated Y value
	 */
	public static double interpolate(double input, SN_Point2D point1, SN_Point2D point2) {
		final double slope = (point2.getY() - point1.getY()) / (point2.getX() - point1.getX());
		final double delta_x = input - point1.getX();
		final double delta_y = delta_x * slope;
		return point1.getY() + delta_y;
	}

	/**
	 * Get X values
	 *
	 * @return X values
	 */
	public double[] getX() {
		double[] xVals = new double[size];
		for (int i = 0; i < size; i++) {
			xVals[i] = m_points[i].getX();
		}
		return xVals;
	}

	/**
	 * Get Y Values
	 *
	 * @return Y Values
	 */
	public double[] getY() {
		double[] yVals = new double[size];
		for (int i = 0; i < size; i++) {
			yVals[i] = m_points[i].getY();
		}
		return yVals;
	}

	/**
	 * Get list of points
	 *
	 * @return list of points
	 */
	public SN_Point2D[] getTable() {
		return m_points;
	}
}
