package com.frcteam3255.preferences;

/**
 * SuperNURDs false preference base class
 */
public class SN_FalsePreference extends SN_BooleanPreference {

	/**
	 * Creates a preference with name "falsePreference" and value 0
	 */
	public SN_FalsePreference() {
		super("falsePreference", false);
	}

	/**
	 * @return false, type boolean
	 */
	@Override
	public boolean getValue() {
		return false;
	}
}
