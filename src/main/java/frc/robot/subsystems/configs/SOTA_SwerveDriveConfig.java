package frc.robot.subsystems.configs;

public class SOTA_SwerveDriveConfig {
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
     * P constant for swerve module angle pid controller
     */
    private double angleP;

    /**
     * I constant for swerve module angle pid controller
     */
    private double angleI;

    /**
     * D constant for swerve module angle pid controller
     */
    private double angleD;

    /**
     * P constant for swerve module speed pid controller
     */
    private double speedP;

    /**
     * I constant for swerve module speed pid controller
     */
    private double speedI;

    /**
     * D constant for swerve module speed pid controller
     */
    private double speedD;

    /**
     * Kv constant for swerve speed feedforward
     */
    private double speedKv;

    /**
     * Ks constant for swerve speed feedforward
     */
    private double speedKs;

    /**
     * Circumfrence of the swerve wheels in inches
     */
    private double wheelCircumfrence;

    /**
     * Ratio between the speed motor and the wheel output
     */
    private double gearRatio;

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

    public double getAngleP() {
        return this.angleP;
    }

    public double getAngleI() {
        return this.angleI;
    }

    public double getAngleD() {
        return this.angleD;
    }

    public double getSpeedP() {
        return this.speedP;
    }

    public double getSpeedI() {
        return this.speedI;
    }

    public double getSpeedD() {
        return this.speedD;
    }

    public double getSpeedKv() {
        return speedKv;
    }

    public double getSpeedKs() {
        return speedKs;
    }

    public double getWheelCircumfrence() {
        return wheelCircumfrence;
    }

    public double getGearRatio() {
        return this.gearRatio;
    }

}
