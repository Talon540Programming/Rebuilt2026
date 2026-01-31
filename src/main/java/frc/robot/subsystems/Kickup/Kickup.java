package frc.robot.subsystems.Kickup;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Kickup.KickupIO.KickupIOInputs;

/**
 * Kickup subsystem for indexing FUEL into the shooter.
 * 
 * The kickup wheels feed FUEL from the hopper into the shooter mechanism.
 * Uses one X60 Kraken motor with MotionMagic velocity control for smooth and
 * fast feeding that works in tandem with the shooter.
 */
public class Kickup extends SubsystemBase {
    
    private final KickupIO io;
    private final KickupIOInputs inputs = new KickupIOInputs();
    
    /**
     * Kickup state machine states
     */
    public enum KickupState {
        /** Kickup wheels are not running */
        NOT_RUNNING,
        /** Kickup wheels are actively feeding FUEL */
        RUNNING
    }
    
    private KickupState currentState = KickupState.NOT_RUNNING;
    
    /**
     * Creates a new Kickup subsystem.
     * 
     * @param io The hardware interface implementation (real or simulated)
     */
    public Kickup(KickupIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        // Update and log inputs
        io.updateInputs(inputs);
        
        // Log state machine info
        Logger.recordOutput("Kickup/CurrentState", currentState.toString());
        Logger.recordOutput("Kickup/AtTargetVelocity", isAtTargetVelocity());
        Logger.recordOutput("Kickup/VelocityError", getVelocityError());
    }
    
    /**
     * Runs the kickup wheels at full feed speed.
     * Uses MotionMagic velocity control for smooth acceleration.
     * Sets state to RUNNING.
     */
    public void feed() {
        currentState = KickupState.RUNNING;
        io.setVelocity(KickupConstants.FEED_VELOCITY_RPS);
    }
    
    /**
     * Runs the kickup wheels at slow feed speed.
     * Uses MotionMagic velocity control for smooth acceleration.
     * Sets state to RUNNING.
     */
    public void slowFeed() {
        currentState = KickupState.RUNNING;
        io.setVelocity(KickupConstants.SLOW_FEED_VELOCITY_RPS);
    }
    
    /**
     * Sets the kickup to a custom velocity using MotionMagic control.
     * Positive values feed FUEL into shooter.
     * Sets state to RUNNING.
     * 
     * @param velocityRPS Target velocity in rotations per second
     */
    public void setVelocity(double velocityRPS) {
        currentState = KickupState.RUNNING;
        io.setVelocity(velocityRPS);
    }
    
    /**
     * Stops the kickup wheels.
     * Sets motor output to neutral and state to NOT_RUNNING.
     */
    public void stop() {
        currentState = KickupState.NOT_RUNNING;
        io.stop();
    }
    
    /**
     * Sets the voltage output for the kickup (open-loop control).
     * Primarily used for characterization and testing.
     * 
     * @param volts Voltage to apply (-12 to +12V)
     */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
    
    /**
     * Gets the current state of the kickup.
     * 
     * @return Current KickupState
     */
    public KickupState getCurrentState() {
        return currentState;
    }
    
    /**
     * Checks if the kickup wheels are at their target velocity within tolerance.
     * 
     * @return true if wheels are within tolerance of target
     */
    public boolean isAtTargetVelocity() {
        double error = Math.abs(inputs.velocityRPS - inputs.targetVelocityRPS);
        return error < KickupConstants.VELOCITY_TOLERANCE;
    }
    
    /**
     * Gets the velocity error for the kickup wheels.
     * 
     * @return Velocity error in rotations per second
     */
    public double getVelocityError() {
        return inputs.targetVelocityRPS - inputs.velocityRPS;
    }
    
    /**
     * Gets the current velocity of the kickup wheels.
     * 
     * @return Current velocity in rotations per second
     */
    public double getVelocity() {
        return inputs.velocityRPS;
    }
    
    /**
     * Gets the target velocity of the kickup wheels.
     * 
     * @return Target velocity in rotations per second
     */
    public double getTargetVelocity() {
        return inputs.targetVelocityRPS;
    }
}