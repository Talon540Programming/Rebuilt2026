package frc.robot.subsystems.Shooter.Hood;

public interface HoodIO {
    
    public static class HoodIOInputs {
        public double positionRotations = 0.0;
        public double positionRadians = 0.0;
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
        public double targetPositionRadians = 0.0;
        public double targetMotorRotations = 0.0;
        public boolean atSetpoint = false;
    }
    
    /** Update the set of loggable inputs */
    public default void updateInputs(HoodIOInputs inputs) {}
    
    /** Set the hood to a target angle in radians */
    public default void setPosition(double positionRadians) {}
    
    /** Run the hood at a duty cycle from -1 to 1 */
    public default void setDutyCycle(double dutyCycle) {}
    
    /** Stop the hood motor */
    public default void stop() {}
    
    /** Zero the hood encoder (call at beginning of auto/teleop) */
    public default void zeroEncoder() {}
    
    /** Enable or disable brake mode */
    public default void setBrakeMode(boolean brake) {}

    /** Sets the current position (in mechanism rotations). Used for zeroing after homing. */
    default void setEncoderPosition(double positionRotations) {}

    /** Enable or disable software limits (disable during homing) */
    default void setSoftLimitsEnabled(boolean enabled) {}
}