package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

/**
 * IO interface for the Intake subsystem.
 * 
 * This interface defines the hardware abstraction layer for the intake roller mechanism.
 * Implementations include real hardware (TalonFX) and simulation.
 */
public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        /** Current velocity of the intake rollers in rotations per second */
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
     * Updates the set of loggable inputs for the intake.
     * This method is called periodically by the subsystem.
     * 
     * @param inputs The inputs object to update with current values
     */
    public default void updateInputs(IntakeIOInputs inputs) {}
    
    /**
     * Sets the target velocity for the intake rollers using MotionMagic velocity control.
     * Positive velocity intakes FUEL, negative velocity ejects FUEL.
     * 
     * @param velocityRPS Target velocity in rotations per second
     */
    public default void setVelocity(double velocityRPS) {}
    
    /**
     * Stops the intake motor.
     * Sets motor output to zero.
     */
    public default void stop() {}
    
    /**
     * Sets the voltage output for the intake (open-loop control).
     * Used primarily for characterization and testing.
     * 
     * @param volts Voltage to apply to motor (-12 to +12V)
     */
    public default void setVoltage(double volts) {}
}