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
		pipeline, off, blink, on
	}

	private double getLEDModeFromEnum(LEDMode a_LEDMode) {
		switch (a_LEDMode) {
			case pipeline :
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
	 * @param a_LEDMode
	 *            Sets the LED Mode on the Limelight
	 */
	public void setLEDMode(LEDMode a_LEDMode) {
		double b_LEDMode = getLEDModeFromEnum(a_LEDMode);

		table.getEntry("ledMode").setDouble(b_LEDMode);
	}

	public LEDMode getLEDMode() {
		switch((int) table.getEntry("ledMode").getDouble(0)) {
			case 0:
				return LEDMode.pipeline;
			case 1:
				return LEDMode.off;
			case 2:
				return LEDMode.blink;
			case 3:
				return LEDMode.on;
			default:
				return LEDMode.pipeline;

		} 
	}

	/**
	 * Decreases exposure, enables vision processing
	 */
	public void setVideoFeedProcessed() {
		table.getEntry("camMode").setDouble(0);
	}

	/**
	 * Increases exposure, disables vision processing
	 */
	public void setVideoFeedNoProcessing() {
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
