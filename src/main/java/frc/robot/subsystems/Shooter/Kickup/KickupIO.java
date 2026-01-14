package frc.robot.subsystems.Shooter.Kickup;

public interface KickupIO {
    
    public static class KickupIOInputs {
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
    }
    
    /** Update the set of loggable inputs */
    public default void updateInputs(KickupIOInputs inputs) {}
    
    /** Run the kickup at a duty cycle from -1 to 1 */
    public default void setDutyCycle(double dutyCycle) {}
    
    /** Stop the kickup motor */
    public default void stop() {}
    
    /** Enable or disable brake mode */
    public default void setBrakeMode(boolean brake) {}
}