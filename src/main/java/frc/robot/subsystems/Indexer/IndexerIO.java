package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;


/**
 * IO interface for the Indexer subsystem.
 * 
 * This interface defines the hardware abstraction layer for the hopper indexing mechanism.
 * Implementations include real hardware (TalonFX) and simulation.
 */
public interface IndexerIO {
    
    @AutoLog
    public static class IndexerIOInputs {
        /** Current velocity of the indexer in rotations per second */
        public double velocityRPS = 0.0;
        
        /** Applied voltage to the motor (V) */
        public double appliedVolts = 0.0;
        
        /** Supply current draw (A) */
        public double supplyCurrentAmps = 0.0;
        
        /** Stator current draw (A) */
        public double statorCurrentAmps = 0.0;
        
        /** Motor temperature (Celsius) */
        public double tempCelsius = 0.0;
        
        /** Target velocity (rps) */
        public double targetVelocityRPS = 0.0;
    }
    
    /**
     * Updates the set of loggable inputs for the indexer.
     * This method is called periodically by the subsystem.
     * 
     * @param inputs The inputs object to update with current values
     */
    public default void updateInputs(IndexerIOInputs inputs) {}
    
    /**
     * Sets the target velocity for the indexer using MotionMagic velocity control.
     * Positive velocity indexes FUEL forward, negative velocity reverses.
     * 
     * @param velocityRPS Target velocity in rotations per second
     */
    public default void setVelocity(double velocityRPS) {}
    
    /**
     * Stops the indexer motor.
     * Sets motor output to zero.
     */
    public default void stop() {}
    
    /**
     * Sets the voltage output for the indexer (open-loop control).
     * Used primarily for characterization and testing.
     * 
     * @param volts Voltage to apply to motor (-12 to +12V)
     */
    public default void setVoltage(double volts) {}
}