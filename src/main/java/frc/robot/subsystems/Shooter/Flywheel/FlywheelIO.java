package frc.robot.subsystems.Shooter.Flywheel;

public interface FlywheelIO {
    
    public static class FlywheelIOInputs {
        public double velocityRotPerSec = 0.0;
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
        public double targetVelocityRPM = 0.0;
        public boolean atSetpoint = false;
    }
    
    public enum FlywheelControlMode {
        COAST,
        DUTY_CYCLE_BANG_BANG,
        TORQUE_CURRENT_BANG_BANG
    }
    
    /** Update the set of loggable inputs */
    public default void updateInputs(FlywheelIOInputs inputs) {}
    
    /** Run the flywheel with bang-bang control at a target velocity in RPM */
    public default void runBangBang(double velocityRPM, FlywheelControlMode mode) {}
    
    /** Run the flywheel at a target velocity in RPM (legacy Motion Magic) */
    public default void setVelocity(double velocityRPM) {}
    
    /** Run the flywheel at a duty cycle from -1 to 1 */
    public default void setDutyCycle(double dutyCycle) {}
    
    /** Stop the flywheel */
    public default void stop() {}
    
    /** Enable or disable brake mode */
    public default void setBrakeMode(boolean brake) {}
}