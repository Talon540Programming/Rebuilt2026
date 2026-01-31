package frc.robot.subsystems.Wrist;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Wrist.WristIO.WristIOInputs;

/**
 * Wrist subsystem for controlling the intake rotation mechanism.
 * 
 * The wrist controls the position of the over-the-bumper intake mechanism using
 * one X60 Kraken motor with MotionMagic position control. It smoothly moves the
 * intake between stowed (inside robot), retracting, and deploying positions.
 * 
 * The encoder is zeroed at match start using a hard stop position.
 */
public class Wrist extends SubsystemBase {
    
    private final WristIO io;
    private final WristIOInputs inputs = new WristIOInputs();
    
    /**
     * Wrist state machine states
     */
    public enum WristState {
        /** Wrist is stowed inside the robot frame */
        STOWED,
        /** Wrist is actively retracting into the robot */
        RETRACTING,
        /** Wrist is actively deploying out of the robot */
        DEPLOYING
    }
    
    private WristState currentState = WristState.STOWED;

    /**
     * Creates a new Wrist subsystem.
     * 
     * @param io The hardware interface implementation (real or simulated)
     */
    public Wrist(WristIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        // Update and log inputs
        io.updateInputs(inputs);
        
        // Log state machine info
        Logger.recordOutput("Wrist/CurrentState", currentState.toString());
        Logger.recordOutput("Wrist/AtTargetPosition", isAtTargetPosition());
        Logger.recordOutput("Wrist/PositionError", getPositionError());
        Logger.recordOutput("Wrist/IsStowed", isStowed());
        Logger.recordOutput("Wrist/IsDeployed", isDeployed());
        
        // Update state machine
        updateStateMachine();
    }
    
    /**
     * Updates the state machine based on current state and position.
     */
    private void updateStateMachine() {
        switch (currentState) {
            case STOWED:
                // Already at stowed position, do nothing
                break;
                
            case RETRACTING:
                // Check if we've reached stowed position
                if (isStowed()) {
                    currentState = WristState.STOWED;
                }
                break;
                
            case DEPLOYING:
                // Continue deploying until commanded otherwise
                // State will change when stow() or retract() is called
                break;
        }
    }
    
    /**
     * Commands the wrist to stow (retract to inside robot).
     * Sets state to RETRACTING and commands position to MIN_ANGLE.
     */
    public void stow() {
        currentState = WristState.RETRACTING;
        io.setPosition(WristConstants.MIN_ANGLE_ROTATIONS);
    }
    
    /**
     * Commands the wrist to deploy (extend over bumper).
     * Sets state to DEPLOYING and commands position to MAX_ANGLE.
     */
    public void deploy() {
        currentState = WristState.DEPLOYING;
        io.setPosition(WristConstants.MAX_ANGLE_ROTATIONS);
    }
    
    /**
     * Alias for stow() - retracts wrist into robot.
     */
    public void retract() {
        stow();
    }
    
    /**
     * Sets the wrist to a specific position using MotionMagic control.
     * Position is automatically clamped to min/max limits.
     * Updates state based on direction of movement.
     * 
     * @param positionRotations Target position in rotations
     */
    public void setPosition(double positionRotations) {
        // Determine state based on movement direction
        if (positionRotations < inputs.positionRotations) {
            currentState = WristState.RETRACTING;
        } else if (positionRotations > inputs.positionRotations) {
            currentState = WristState.DEPLOYING;
        }
        
        io.setPosition(positionRotations);
    }
    
    /**
     * Zeros the wrist encoder at the current position.
     * This should be called when the wrist is at a known position (e.g., hard stop).
     * Typically called during robot initialization when wrist is at stowed position.
     */
    public void zeroEncoder() {
        io.zeroEncoder();
        currentState = WristState.STOWED;
    }
    
    /**
     * Stops the wrist motor.
     * Holds the current position when using brake mode.
     */
    public void stop() {
        io.stop();
    }
    
    /**
     * Sets the voltage output for the wrist (open-loop control).
     * Primarily used for characterization and testing.
     * 
     * @param volts Voltage to apply (-12 to +12V)
     */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
    
    // ========== State Queries ==========
    
    /**
     * Gets the current state of the wrist.
     * 
     * @return Current WristState
     */
    public WristState getCurrentState() {
        return currentState;
    }
    
    /**
     * Checks if the wrist is at its target position within tolerance.
     * 
     * @return true if wrist is within tolerance of target
     */
    public boolean isAtTargetPosition() {
        double error = Math.abs(inputs.positionRotations - inputs.targetPositionRotations);
        return error < WristConstants.POSITION_TOLERANCE;
    }
    
    /**
     * Gets the position error for the wrist.
     * 
     * @return Position error in rotations
     */
    public double getPositionError() {
        return inputs.targetPositionRotations - inputs.positionRotations;
    }
    
    /**
     * Gets the current position of the wrist.
     * 
     * @return Current position in rotations
     */
    public double getPosition() {
        return inputs.positionRotations;
    }
    
    /**
     * Gets the target position of the wrist.
     * 
     * @return Target position in rotations
     */
    public double getTargetPosition() {
        return inputs.targetPositionRotations;
    }
    
    /**
     * Checks if the wrist is stowed (at minimum position).
     * 
     * @return true if wrist is stowed
     */
    public boolean isStowed() {
        return Math.abs(inputs.positionRotations - WristConstants.MIN_ANGLE_ROTATIONS) 
               < WristConstants.POSITION_TOLERANCE;
    }
    
    /**
     * Checks if the wrist is deployed (at maximum position).
     * 
     * @return true if wrist is deployed
     */
    public boolean isDeployed() {
        return Math.abs(inputs.positionRotations - WristConstants.MAX_ANGLE_ROTATIONS) 
               < WristConstants.POSITION_TOLERANCE;
    }
}