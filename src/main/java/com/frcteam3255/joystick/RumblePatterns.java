// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.frcteam3255.joystick;


import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class RumblePatterns {
    private final RumbleType rumbleType;
    private final double intensity;

    public RumblePatterns(RumbleType rumbleType, double intensity) {
        this.rumbleType = rumbleType;
        this.intensity = intensity;
    }

    public static RumblePatterns getRumblePattern(RumbleType rumbleType, double rumbleIntensity) {
        // Replace this with your preference system as needed; using a plain String for compilation
        String selectedRumble = "Steady";
        double t = Timer.getFPGATimestamp(); // seconds since FPGA boot
        boolean toggle = ((int) Math.floor(t) % 2) == 0; // toggle every 1 second
        final RumblePatterns steadyController = new RumblePatterns(rumbleType, rumbleIntensity);
        final RumblePatterns buzzController = new RumblePatterns(rumbleType, rumbleIntensity);
        final RumblePatterns flickerController = new RumblePatterns(rumbleType, toggle ? rumbleIntensity : 0);

        if ("Steady".equals(selectedRumble)) {
            return steadyController;
        } else if ("Buzz".equals(selectedRumble)) {
            return buzzController;
        } else if ("Flicker".equals(selectedRumble)) {
            return flickerController;
        }

        // default
        return steadyController;
    }
}
