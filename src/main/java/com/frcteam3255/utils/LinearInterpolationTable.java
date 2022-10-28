package com.frcteam3255.utils;

/**
 * Linear Interpolation Table for finding the best value based on limited data points. "Borrowed" from 1706's 2022 code.
 */
public class LinearInterpolationTable {
    private double m_maxInput = Double.NEGATIVE_INFINITY;
    private double m_minInput = Double.POSITIVE_INFINITY;
    private final SN_Point2D[] m_points;
    public final int size;

    /**
     * Create new SN_Lerp 
     * @param points
     */
    public LinearInterpolationTable(SN_Point2D... points) {
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

    public static double interpolate(double input, SN_Point2D point1, SN_Point2D point2) {
        final double slope = (point2.getY() - point1.getY()) / (point2.getX() - point1.getX());
        final double delta_x = input - point1.getX();
        final double delta_y = delta_x * slope;
        return point1.getY() + delta_y;
    }

    public double[] getX() {
        double[] xVals = new double[size];
        for (int i = 0; i < size; i++) {
            xVals[i] = m_points[i].getX();
        }
        return xVals;
    }

    public double[] getY() {
        double[] yVals = new double[size];
        for (int i = 0; i < size; i++) {
            yVals[i] = m_points[i].getY();
        }
        return yVals;
    }

    public SN_Point2D[] getTable() {
        return m_points;
    }
}
