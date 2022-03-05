package com.frcteam3255.preferences;

/**
 * SuperNURDs true preference base class
 */
public class SN_TruePreference extends SN_BooleanPreference {

	/**
	 * Creates a preference with name "truePreference" and value 0
	 */
	public SN_TruePreference() {
		super("truePreference", true);
	}

	/**
	 * @return true, type boolean
	 */
	@Override
	public boolean getValue() {
		return true;
	}
}
