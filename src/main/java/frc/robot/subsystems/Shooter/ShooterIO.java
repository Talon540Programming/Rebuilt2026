package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;


/**
 * IO interface for the Shooter subsystem.
 * 
 * This interface defines the hardware abstraction layer for the shooter flywheel.
 * Implementations include real hardware (TalonFX) and simulation.
 */
public interface ShooterIO {
    
    @AutoLog
    public static class ShooterIOInputs {
        /** Bottom flywheel velocity in rotations per second */
        public double bottomVelocityRPS = 0.0;
        
        /** Top flywheel velocity in rotations per second */
        public double topVelocityRPS = 0.0;
        
        /** Bottom flywheel applied voltage (V) */
        public double bottomAppliedVolts = 0.0;
        
        /** Top flywheel applied voltage (V) */
        public double topAppliedVolts = 0.0;
        
        /** Bottom flywheel supply current (A) */
        public double bottomSupplyCurrentAmps = 0.0;
        
        /** Top flywheel supply current (A) */
        public double topSupplyCurrentAmps = 0.0;
        
        /** Bottom flywheel stator current (A) */
        public double bottomStatorCurrentAmps = 0.0;
        
        /** Top flywheel stator current (A) */
        public double topStatorCurrentAmps = 0.0;
        
        /** Bottom flywheel temperature (Celsius) */
        public double bottomTempCelsius = 0.0;
        
        /** Top flywheel temperature (Celsius) */
        public double topTempCelsius = 0.0;
        
        /** Target velocity for bottom flywheel (rps) */
        public double targetBottomVelocityRPS = 0.0;
        
        /** Target velocity for top flywheel (rps) */
        public double targetTopVelocityRPS = 0.0;
    }
    
    /**
     * Updates the set of loggable inputs for the shooter.
     * This method is called periodically by the subsystem.
     * 
     * @param inputs The inputs object to update with current values
     */
    public default void updateInputs(ShooterIOInputs inputs) {}
    
    /**
     * Sets the target velocity for both flywheels using closed-loop control.
     * The bottom flywheel spins at the target velocity, while the top wheel
     * spins 3x faster due to gear ratio difference.
     * 
     * @param bottomVelocityRPS Target velocity for the bottom flywheel in rotations per second
     */
    public default void setVelocity(double bottomVelocityRPS) {}
    
    /**
     * Stops both flywheel motors.
     * Sets motor output to zero.
     */
    public default void stop() {}
    
    /**
     * Sets the voltage output for both flywheels (open-loop control).
     * Used primarily for characterization and testing.
     * 
     * @param volts Voltage to apply to motors (-12 to +12V)
     */
    public default void setVoltage(double volts) {}
}