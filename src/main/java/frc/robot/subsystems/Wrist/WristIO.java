package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Wrist subsystem.
 * 
 * This interface defines the hardware abstraction layer for the 4-bar linkage rotation.
 * Implementations include real hardware (TalonFX) and simulation.
 */
public interface WristIO {
    
    @AutoLog
    public static class WristIOInputs {
        /** Current position of the wrist in rotations */
        public double positionRotations = 0.0;
        
        /** Current velocity of the wrist in rotations per second */
        public double velocityRPS = 0.0;
        
        /** Applied voltage to the motor (V) */
        public double appliedVolts = 0.0;
        
        /** Supply current draw (A) */
        public double supplyCurrentAmps = 0.0;
        
        /** Stator current draw (A) */
        public double statorCurrentAmps = 0.0;
        
        /** Motor temperature (Celsius) */
        public double tempCelsius = 0.0;
        
        /** Target position (rotations) */
        public double targetPositionRotations = 0.0;
    }
    
    /**
     * Updates the set of loggable inputs for the wrist.
     * This method is called periodically by the subsystem.
     * 
     * @param inputs The inputs object to update with current values
     */
    public default void updateInputs(WristIOInputs inputs) {}
    
    /**
     * Sets the target position for the wrist using MotionMagic position control.
     * The position is automatically clamped between min and max limits.
     * 
     * @param positionRotations Target position in rotations
     */
    public default void setPosition(double positionRotations) {}
    
    /**
     * Zeros the wrist encoder at the current position.
     * This should be called when the wrist is at a known position (e.g., hard stop).
     */
    public default void zeroEncoder() {}
    
    /**
     * Stops the wrist motor.
     * Sets motor output to zero.
     */
    public default void stop() {}
    
    /**
     * Sets the voltage output for the wrist (open-loop control).
     * Used primarily for characterization and testing.
     * 
     * @param volts Voltage to apply to motor (-12 to +12V)
     */
    public default void setVoltage(double volts) {}
}