package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer.IndexerIO.IndexerIOInputs;

/**
 * Indexer subsystem for moving FUEL through the hopper.
 * 
 * The indexer consists of belts and rollers in the hopper controlled by one X44 motor.
 * Uses MotionMagic velocity control for smooth acceleration and consistent
 * indexing speed when moving FUEL from the intake to the kickup wheels.
 * 
 * Includes TimeOfFlight sensor for fuel counting.
 */
public class Indexer extends SubsystemBase {
    
    private final IndexerIO io;
    private final IndexerIOInputs inputs = new IndexerIOInputs();
    private final IndexerSensor sensor;
    
    /**
     * Indexer state machine states
     */
    public enum IndexerState {
        /** Indexer is not moving FUEL */
        NOT_INDEXING,
        /** Indexer is actively moving FUEL */
        INDEXING
    }
    
    private IndexerState currentState = IndexerState.NOT_INDEXING;
    
    /**
     * Creates a new Indexer subsystem.
     * 
     * @param io The hardware interface implementation (real or simulated)
     * @param sensor The TimeOfFlight sensor for fuel counting
     */
    public Indexer(IndexerIO io, IndexerSensor sensor) {
        this.io = io;
        this.sensor = sensor;
    }
    
    /**
     * Creates a new Indexer subsystem without sensor (for simulation).
     * 
     * @param io The hardware interface implementation
     */
    public Indexer(IndexerIO io) {
        this.io = io;
        this.sensor = null; // No sensor in simulation
    }
    
    @Override
    public void periodic() {
        // Update and log inputs
        io.updateInputs(inputs);
        
        // Update sensor if present
        if (sensor != null) {
            sensor.update();
        }
        
        // Log state machine info
        Logger.recordOutput("Indexer/CurrentState", currentState.toString());
        Logger.recordOutput("Indexer/AtTargetVelocity", isAtTargetVelocity());
        Logger.recordOutput("Indexer/VelocityError", getVelocityError());
    }
    
    /**
     * Runs the indexer at the configured indexing speed.
     * Moves FUEL forward through the hopper.
     * Uses MotionMagic velocity control for smooth acceleration.
     * Sets state to INDEXING.
     */
    public void index() {
        currentState = IndexerState.INDEXING;
        io.setVelocity(IndexerConstants.INDEX_VELOCITY_RPS);
    }
    
    /**
     * Runs the indexer at the configured feeding speed.
     * Feeds FUEL into the shooter at higher speed.
     * Uses MotionMagic velocity control for smooth acceleration.
     * Sets state to INDEXING.
     */
    public void feed() {
        currentState = IndexerState.INDEXING;
        io.setVelocity(IndexerConstants.FEED_VELOCITY_RPS);
    }
    
    /**
     * Runs the indexer in reverse to eject FUEL.
     * Uses MotionMagic velocity control for smooth acceleration.
     * Sets state to INDEXING.
     */
    public void reverse() {
        currentState = IndexerState.INDEXING;
        io.setVelocity(IndexerConstants.REVERSE_VELOCITY_RPS);
    }
    
    /**
     * Sets the indexer to a custom velocity using MotionMagic control.
     * Positive values move FUEL forward, negative values reverse.
     * Sets state to INDEXING.
     * 
     * @param velocityRPS Target velocity in rotations per second
     */
    public void setVelocity(double velocityRPS) {
        currentState = IndexerState.INDEXING;
        io.setVelocity(velocityRPS);
    }
    
    /**
     * Stops the indexer.
     * Sets motor output to neutral and state to NOT_INDEXING.
     */
    public void stop() {
        currentState = IndexerState.NOT_INDEXING;
        io.stop();
    }
    
    /**
     * Sets the voltage output for the indexer (open-loop control).
     * Primarily used for characterization and testing.
     * 
     * @param volts Voltage to apply (-12 to +12V)
     */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
    
    /**
     * Gets the current state of the indexer.
     * 
     * @return Current IndexerState
     */
    public IndexerState getCurrentState() {
        return currentState;
    }
    
    /**
     * Checks if the indexer is at its target velocity within tolerance.
     * 
     * @return true if indexer is within tolerance of target
     */
    public boolean isAtTargetVelocity() {
        double error = Math.abs(inputs.velocityRPS - inputs.targetVelocityRPS);
        return error < IndexerConstants.VELOCITY_TOLERANCE;
    }
    
    /**
     * Gets the velocity error for the indexer.
     * 
     * @return Velocity error in rotations per second
     */
    public double getVelocityError() {
        return inputs.targetVelocityRPS - inputs.velocityRPS;
    }
    
    /**
     * Gets the current velocity of the indexer.
     * 
     * @return Current velocity in rotations per second
     */
    public double getVelocity() {
        return inputs.velocityRPS;
    }
    
    /**
     * Gets the target velocity of the indexer.
     * 
     * @return Target velocity in rotations per second
     */
    public double getTargetVelocity() {
        return inputs.targetVelocityRPS;
    }
    
    /**
     * Gets the fuel sensor.
     * 
     * @return The indexer sensor, or null if not present
     */
    public IndexerSensor getSensor() {
        return sensor;
    }
}