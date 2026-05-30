// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frcteam3255.joystick;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class RumblePatterns {
    public RumblePatterns(RumbleType rumbleType, double rumbleIntensity ){
    double t = Timer.getFPGATimestamp(); // seconds since FPGA boot
    boolean toggle = ((int) Math.floor(t) % 2) == 0; // toggle every 1 second


    final RumblePatterns steadyController = new RumblePatterns(rumbleType, 0);
    final RumblePatterns buzzController = new RumblePatterns(rumbleType, rumbleIntensity);
    final RumblePatterns flickerController = new RumblePatterns(rumbleType, toggle ? rumbleIntensity : 0);
    }
}
