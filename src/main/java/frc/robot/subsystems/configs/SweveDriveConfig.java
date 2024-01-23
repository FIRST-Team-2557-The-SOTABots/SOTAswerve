package frc.robot.subsystems.configs;

public class SweveDriveConfig {
    /**
     * Wheel Base (wheel distance front to back) in inches
     */
    private double wheelBase;

    /**
     * Track Width (wheel distance side to side) in inches
     */
    private double trackWidth;

    /**
     * max speed of the modules in feet per second
     */
    private double maxSpeed;

    /**
     * max rotational speed of the robot in radians per second
     */
    private double maxRotationalVelocity;

    /**
     * @return math rotational velocity of the robot in radians per second
     */
    public double getMaxRotationalVelocity() {
        return maxRotationalVelocity;
    }

    /**
     * @return wheel base in inches
     */
    public double getWheelBase() {
        return wheelBase;
    }

    /**
     * @return track width in inches
     */
    public double getTrackWidth() {
        return trackWidth;
    }

    /**
     * @return max speed of the modules in feet per second
     */
    public double getMaxSpeed() {
        return maxSpeed;
    }
}
