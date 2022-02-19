package com.frcteam3255.preferences;

/**
 * SuperNURDs double zero preference base class
 */
public class SN_ZeroDoublePreference extends SN_DoublePreference {

	/**
	 * Creates a preference with name "doubleZeroPreference" and value 0
	 */
	public SN_ZeroDoublePreference() {
		super("doubleZeroPreference", 0);
	}

	/**
	 * @return 0, type double
	 */
	@Override
	public double getValue() {
		return 0;
	}
}
