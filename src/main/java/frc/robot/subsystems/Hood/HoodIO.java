package frc.robot.subsystems.Hood;

import org.littletonrobotics.junction.AutoLog;


/**
 * IO interface for the Hood subsystem.
 * 
 * This interface defines the hardware abstraction layer for the shooter hood angle control.
 * Implementations include real hardware (TalonFX) and simulation.
 */
public interface HoodIO {
    
    @AutoLog
    public static class HoodIOInputs {
        /** Current position of the hood in rotations */
        public double positionRotations = 0.0;
        
        /** Current velocity of the hood in rotations per second */
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
     * Updates the set of loggable inputs for the hood.
     * This method is called periodically by the subsystem.
     * 
     * @param inputs The inputs object to update with current values
     */
    default void updateInputs(HoodIOInputs inputs) {}
    
    /**
     * Sets the target position for the hood using MotionMagic position control.
     * The position is automatically clamped between min and max limits.
     * 
     * @param positionRotations Target position in rotations
     */
    default void setPosition(double positionRotations) {}
    
    /**
     * Zeros the hood encoder at the current position.
     * This should be called at the beginning of each match when hood is at a known position.
     */
    default void zeroEncoder() {}
    
    /**
     * Stops the hood motor.
     * Sets motor output to zero.
     */
    default void stop() {}
    
    /**
     * Sets the voltage output for the hood (open-loop control).
     * Used primarily for characterization and testing.
     * 
     * @param volts Voltage to apply to motor (-12 to +12V)
     */
    default void setVoltage(double volts) {}
}