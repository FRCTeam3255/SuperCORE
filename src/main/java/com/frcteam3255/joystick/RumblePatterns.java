// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frcteam3255.joystick;


import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class RumblePatterns {
    public final RumbleType rumbleType;
    public final double intensity;

    public RumblePatterns(RumbleType rumbleType, double intensity) {
        this.rumbleType = rumbleType;
        this.intensity = intensity;
    }

    public RumblePatterns getRumblePattern(String selectedRumble) {
        // Replace this with your preference system as needed; using a plain String for compilation
        double t = Timer.getFPGATimestamp(); // seconds since FPGA boot
        boolean toggle = ((int) Math.floor(t) % 2) == 0; // toggle every 1 second
        final RumblePatterns steadyController = new RumblePatterns(rumbleType, 0);
        final RumblePatterns buzzController = new RumblePatterns(rumbleType, intensity);
        final RumblePatterns flickerController = new RumblePatterns(rumbleType, toggle ? intensity : 0);

        if ("Buzz".equals(selectedRumble)) {
            return buzzController;
        } else if ("Flicker".equals(selectedRumble)) {
            return flickerController;
        }
        else {
        return steadyController;
        }
    }
}
