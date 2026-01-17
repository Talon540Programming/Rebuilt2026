package frc.robot.subsystems.Index;


public interface IndexIO {
    
    public static class IndexIOInputs {
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
        
        // CANRange sensor data
        public double distanceMeters = 0.0;
        public boolean hasGamePiece = false;
    }
    
    /** Update the set of loggable inputs */
    public default void updateInputs(IndexIOInputs inputs) {}
    
    /** Run the index at a duty cycle from -1 to 1 */
    public default void setDutyCycle(double dutyCycle) {}
    
    /** Stop the index motor */
    public default void stop() {}
    
    /** Enable or disable brake mode */
    public default void setBrakeMode(boolean brake) {}
}