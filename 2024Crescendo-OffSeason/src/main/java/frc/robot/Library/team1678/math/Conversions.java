package frc.robot.Library.team1678.math;

public class Conversions {

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */
    public static double degreesToFalcon(double degrees, double gearRatio) {
        double ticks =  degrees / (360.0 / (gearRatio * 2048.0));
        return ticks;
    }

    /**
     * @param counts Falcon Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double falconToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 2048.0));
    }

    /**
     * @param degrees Degrees of rotation of Mechanism
     * @param gearRatio Gear Ratio between Talon and Mechanism
     * @return Talon Counts
     */
    public static double degreesToTalon(double degrees, double gearRatio) {
        double ticks =  degrees / (360.0 / (gearRatio * 4096.0));
        return ticks;
    }

    /**
     * @param counts Talon Counts
     * @param gearRatio Gear Ratio between Talon and Mechanism
     * @return Degrees of Rotation of Mechanism
     */
    public static double talonToDegrees(double counts, double gearRatio) {
        return counts * (360.0 / (gearRatio * 4096.0));
    }

    /**
     * @param Radians Radians of rotation of Mechanism
     * @param gearRatio Gear Ratio between Talon and Mechanism
     * @return Talon Counts
     */
    public static double RadiansToTalon(double Radians, double gearRatio) {
        double ticks =  Radians / ((Math.PI * 2) / (gearRatio * 4096.0));
        return ticks;
    }

    /**
     * @param Radians Radians of rotation of Mechanism
     * @param gearRatio Gear Ratio between Talon and Mechanism
     * @return Talon Counts
     */
    public static double TalonToRadians(double counts, double gearRatio) {
        return counts * (2 * Math.PI / (gearRatio * 4096.0));
    }

    /**
     * @param Radians Radians of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */
    public static double RadiansToFalcon(double Radians, double gearRatio) {
        double ticks =  Radians / ((Math.PI * 2) / (gearRatio * 2048.0));
        return ticks;
    }

    /**
     * @param Radians Radians of rotation of Mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism
     * @return Falcon Counts
     */
    public static double FalconToRadians(double counts, double gearRatio) {
        return counts * (2 * Math.PI / (gearRatio * 2048.0));
    }

    /**
     * @param velocityCounts Falcon Velocity Counts
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return RPM of Mechanism
     */
    public static double falconToRPM(double velocityCounts, double gearRatio) {
        double motorRPM = velocityCounts * (600.0 / 2048.0); 
        double mechRPM = motorRPM / gearRatio;
        return mechRPM;
    }

    /**
     * @param RPM RPM of mechanism
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double RPMToFalcon(double RPM, double gearRatio) {
        double motorRPM = RPM * gearRatio;
        double sensorCounts = motorRPM * (2048.0 / 600.0);
        return sensorCounts;
    }

    /**
     * @param velocitycounts Falcon Velocity Counts
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return MPS of Mechanism
     */
    public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
        double wheelRPM = falconToRPM(velocitycounts, gearRatio);
        double wheelMPS = (wheelRPM * circumference) / 60;
        return wheelMPS;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
     * @return Falcon Velocity Counts
     */
    public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
        double wheelRPM = ((velocity * 60) / circumference);
        double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
        return wheelVelocity;
    }

    /**
     * @param velocity Velocity MPS
     * @param circumference Circumference of Wheel
     * @return RPM of Mechanism
     */
    public static double MPSToRPM(double velocity, double circumference) {
        double wheelRPM = ((velocity * 60) / circumference);
        return wheelRPM;
    }

    /**
     * @param velocity Velocity RPM
     * @param circumference Circumference of Wheel
     * @return MPS of Mechanism
     */
    public static double RPMToMPS(double velocity , double circumference) {
        double wheelMPS = ((velocity  / 60) * circumference);
        return wheelMPS;
    }

    // Convert meters to inches
    public static double metersToInches(double meters) {
        return meters * (39.73701 / 1);
    }

    // Convert meters to inches
    public static double inchesToMeters(double inches) {
        return inches * (0.0254 / 1);
    }

}
