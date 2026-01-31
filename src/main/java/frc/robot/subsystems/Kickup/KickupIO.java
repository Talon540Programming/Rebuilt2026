package frc.robot.subsystems.Kickup;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Kickup subsystem.
 * 
 * This interface defines the hardware abstraction layer for the kickup wheel mechanism.
 * Implementations include real hardware (TalonFX) and simulation.
 */
public interface KickupIO {
    
    @AutoLog
    public static class KickupIOInputs {
        /** Current velocity of the kickup wheels in rotations per second */
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
     * Updates the set of loggable inputs for the kickup.
     * This method is called periodically by the subsystem.
     * 
     * @param inputs The inputs object to update with current values
     */
    default void updateInputs(KickupIOInputs inputs) {}
    
    /**
     * Sets the target velocity for the kickup wheels using MotionMagic velocity control.
     * Positive velocity feeds FUEL into the shooter.
     * 
     * @param velocityRPS Target velocity in rotations per second
     */
    default void setVelocity(double velocityRPS) {}
    
    /**
     * Stops the kickup motor.
     * Sets motor output to zero.
     */
    default void stop() {}
    
    /**
     * Sets the voltage output for the kickup (open-loop control).
     * Used primarily for characterization and testing.
     * 
     * @param volts Voltage to apply to motor (-12 to +12V)
     */
    default void setVoltage(double volts) {}
}