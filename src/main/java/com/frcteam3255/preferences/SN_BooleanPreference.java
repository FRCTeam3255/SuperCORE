/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.frcteam3255.preferences;

import edu.wpi.first.wpilibj.Preferences;

/**
 * SuperNURDs boolean preference base class
 */
public class SN_BooleanPreference extends SN_Preferences {
	private boolean m_defaultValue;

	/**
	 * Creates a preference with a name and boolean
	 *
	 * @param name
	 *            - RobotPreferences Key
	 * @param defaultValue
	 *            - Value to default to
	 */
	public SN_BooleanPreference(String name, boolean defaultValue) {
		m_name = name;
		m_defaultValue = defaultValue;
	}

	/**
	 * @return if using defaults return coded values or get new values from network
	 *         tables
	 */
	public boolean getValue() {
		if (isUsingDefaults()) {
			return m_defaultValue;
		}
		return Preferences.getBoolean(m_name, m_defaultValue);
	}
}
