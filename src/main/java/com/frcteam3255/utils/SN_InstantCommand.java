// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frcteam3255.utils;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SN_InstantCommand extends InstantCommand {
	private boolean m_runsWhenDisabled;

	public SN_InstantCommand() {
		super();
	}

	/**
	 * Creates a new InstantCommand that runs the given Runnable with the given
	 * requirements.
	 *
	 * @param toRun
	 *            the Runnable to run
	 * @param requirements
	 *            the subsystems required by this command
	 * @param boolean
	 *            whether the command should runnable when the robot is disabled
	 */
	public SN_InstantCommand(Runnable toRun, boolean runsWhenDisabled, Subsystem... requirements) {
		super(toRun, requirements);
		m_runsWhenDisabled = runsWhenDisabled;
	}

	/**
	 * Creates a new InstantCommand that runs the given Runnable with the given
	 * requirements.
	 *
	 * @param toRun
	 *            the Runnable to run
	 * @param requirements
	 *            the subsystems required by this command
	 */
	public SN_InstantCommand(Runnable toRun, Subsystem... requirements) {
		this(toRun, false, requirements);
	}

	@Override
	public final boolean runsWhenDisabled() {
		return m_runsWhenDisabled;
	}
}
