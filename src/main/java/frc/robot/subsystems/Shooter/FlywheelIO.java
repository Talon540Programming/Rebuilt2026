package frc.robot.subsystems.Shooter;

public interface FlywheelIO {
    
    public static class FlywheelIOInputs {
        public double velocityRotPerSec = 0.0;
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
        public double targetVelocityRPM = 0.0;
        public boolean atSetpoint = false;
    }
    
    /** Update the set of loggable inputs */
    public default void updateInputs(FlywheelIOInputs inputs) {}
    
    /** Run the flywheel at a target velocity in RPM */
    public default void setVelocity(double velocityRPM) {}
    
    /** Run the flywheel at a duty cycle from -1 to 1 */
    public default void setDutyCycle(double dutyCycle) {}
    
    /** Stop the flywheel */
    public default void stop() {}
    
    /** Enable or disable brake mode */
    public default void setBrakeMode(boolean brake) {}
}