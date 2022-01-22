package com.frcteam3255.components;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SN_Limelight {

	NetworkTable table;

	/**
	 * Wrapper for the Limelight
	 * <p>
	 * Adds custom return methods like {@link #hasTarget()} and
	 * {@link #getOffsetX()}
	 */
	public SN_Limelight() {
		table = NetworkTableInstance.getDefault().getTable("limelight");
	}

	/**
	 * @return Whether the limelight has any valid targets
	 */
	public boolean hasTarget() {
		return table.getEntry("tv").getDouble(0) > 0;
	}

	/**
	 * @return Horizontal Offset From Crosshair To Target
	 */
	public double getOffsetX() {
		return table.getEntry("tx").getDouble(0);
	}

	/**
	 * @return Vertical Offset From Crosshair To Target
	 */
	public double getOffsetY() {
		return table.getEntry("ty").getDouble(0);
	}

	/**
	 * @return Target Area Percentage
	 */
	public double getTargetArea() {
		return table.getEntry("ta").getDouble(0);
	}

	public enum LEDMode {
		current, off, blink, on
	}

	private double getLEDMode(LEDMode a_ledMode) {
		switch (a_ledMode) {
			case current :
				return 0;
			case off :
				return 1;
			case blink :
				return 2;
			case on :
				return 3;
			default :
				return 0;
		}
	}

	/**
	 * @param a_ledMode
	 *            Sets the LED Mode on the Limelight
	 */
	public void setLedMode(LEDMode a_ledMode) {
		double b_ledMode = getLEDMode(a_ledMode);

		table.getEntry("ledMode").setDouble(b_ledMode);
	}

	/**
	 * Decreases exposure, enables vision processing
	 */
	public void setCamVisionProcessor() {
		table.getEntry("camMode").setDouble(0);
	}

	/**
	 * Increases exposure, disables vision processing
	 */
	public void setCamDriverCamera() {
		table.getEntry("camMode").setDouble(1);
	}

	/**
	 * Select vision pipeline 0 through 9
	 *
	 * @param a_pipeline
	 *            0 .. 9
	 */
	public void setPipeline(int a_pipeline) {
		table.getEntry("pipeline").setDouble(a_pipeline);
	}
}
