package com.frcteam364;

public class Conversions {

    /**
     * Converts Falcon integrated encoder counts (falcon) to degrees
     * 
     * @param counts Falcon Integrated Encoder Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * Converts degrees to Falcon integrated encoder counts (falcon)
     * 
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Integrated Encoder Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        double ticks =  degrees / (360.0 / (gearRatio * 2048.0));
        return ticks;
    }

    /**
     * Converts Falcon integrated encoder counts per 100 milliseconds (falcon) to RPM
     * 
     * @param velocityCounts Falcon Integrated Encoder Counts per 100 milliseconds
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0);        
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * Converts RPM to Falcon integrated encoder counts per 100 milliseconds (falcon) 
     * 
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Integrated Encoder Counts per 100 milliseconds
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * Converts Falcon integrated encoder counts per 100 milliseconds (falcon) to Meters per Second (MPS)
     * 
     * @param velocitycounts Falcon Integrated Encoder Counts per 100 milliseconds
     * @param circumference Circumference of Wheel in Meters
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Mechanism Meters per Second
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * Converts Meters per Second (MPS) to Falcon integrated encoder counts per 100 milliseconds (falcon) 
     * 
     * @param velocity Velocity in Meters per Second
     * @param circumference Circumference of Wheel in Meters
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Integrated Encoder Counts per 100 milliseconds
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

}