package frc.robot.subsystems.Shooter.Flywheel;

public interface FlywheelIO {
    
    /** Flywheel bang-bang control states matching MA's 4-phase approach */
    public enum FlywheelState {
        COAST,      // Not running
        STARTUP,    // Spinning up - duty cycle bang-bang (max power)
        IDLE,       // At speed, waiting - torque current bang-bang (consistent torque)
        RECOVERY    // After shot, recovering speed - duty cycle bang-bang (max power)
    }
    
    public static class FlywheelIOInputs {
        public double velocityRotPerSec = 0.0;
        public double velocityRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double torqueCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
        public double targetVelocityRPM = 0.0;
        public boolean atSetpoint = false;
        public FlywheelState state = FlywheelState.COAST;
        public long shotCount = 0;
    }
    
    /** Update the set of loggable inputs */
    public default void updateInputs(FlywheelIOInputs inputs) {}
    
    /** Run the flywheel with bang-bang control at a target velocity in RPM.
     *  Mode switching (duty cycle vs torque current) is handled internally. */
    public default void runBangBang(double velocityRPM) {}
    
    /** Stop the flywheel */
    public default void stop() {}
    
    /** Enable or disable brake mode */
    public default void setBrakeMode(boolean brake) {}

    public default void simulateShot() {}
}