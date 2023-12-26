package com.frcteam3255.utils;

/**
 * Point in 2D space
 */
public class SN_Point2D {

	double m_x;
	double m_y;

	/**
	 * Create new SN_Point2D
	 *
	 * @param x
	 *            X coordinate
	 * @param y
	 *            Y coordinate
	 */
	public SN_Point2D(double x, double y) {
		m_x = x;
		m_y = y;
	}

	/**
	 * @return X coordinate of the point
	 */
	public double getX() {
		return m_x;
	}

	/**
	 * @return Y coordinate of the point
	 */
	public double getY() {
		return m_y;
	}

}
