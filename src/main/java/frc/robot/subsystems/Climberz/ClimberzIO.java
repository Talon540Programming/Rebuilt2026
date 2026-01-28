package frc.robot.subsystems.Climberz;

public interface ClimberzIO {
    
    public static class ClimberzIOInputs {
        public double positionRotations = 0.0;
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
    }
    
    /** Update the set of loggable inputs */
    public default void updateInputs(ClimberzIOInputs inputs) {}
    
    /** Run climber at the specified duty cycle (-1 to 1) */
    public default void setDutyCycle(double dutyCycle) {}
    
    /** Stop the climber */
    public default void stop() {}
    
    /** Enable or disable brake mode */
    public default void setBrakeMode(boolean brake) {}
}