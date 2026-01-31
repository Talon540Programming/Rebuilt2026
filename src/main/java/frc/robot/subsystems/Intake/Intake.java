package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeIO.IntakeIOInputs;

/**
 * Intake subsystem for collecting FUEL from the ground.
 * 
 * The intake consists of roller wheels controlled by one X44 motor.
 * Uses MotionMagic velocity control for smooth acceleration and consistent
 * roller speed during intaking and ejecting operations.
 */
public class Intake extends SubsystemBase {
    
    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();
    
    /**
     * Intake state machine states
     */
    public enum IntakeState {
        /** Intake rollers are stopped */
        STOWED,
        /** Intake rollers are spinning to collect FUEL */
        INTAKING
    }
    
    private IntakeState currentState = IntakeState.STOWED;
    
    /**
     * Creates a new Intake subsystem.
     * 
     * @param io The hardware interface implementation (real or simulated)
     */
    public Intake(IntakeIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        // Update and log inputs
        io.updateInputs(inputs);
        
        // Log state machine info
        Logger.recordOutput("Intake/CurrentState", currentState.toString());
        Logger.recordOutput("Intake/AtTargetVelocity", isAtTargetVelocity());
        Logger.recordOutput("Intake/VelocityError", getVelocityError());
    }
    
    /**
     * Runs the intake rollers at the configured intake speed.
     * Uses MotionMagic velocity control for smooth acceleration.
     * Sets state to INTAKING.
     */
    public void intake() {
        currentState = IntakeState.INTAKING;
        io.setVelocity(IntakeConstants.INTAKE_VELOCITY_RPS);
    }
    
    /**
     * Runs the intake rollers in reverse to eject FUEL.
     * Uses MotionMagic velocity control for smooth acceleration.
     * Sets state to INTAKING.
     */
    public void eject() {
        currentState = IntakeState.INTAKING;
        io.setVelocity(IntakeConstants.EJECT_VELOCITY_RPS);
    }
    
    /**
     * Sets the intake to a custom velocity using MotionMagic control.
     * Positive values intake, negative values eject.
     * Sets state to INTAKING.
     * 
     * @param velocityRPS Target velocity in rotations per second
     */
    public void setVelocity(double velocityRPS) {
        currentState = IntakeState.INTAKING;
        io.setVelocity(velocityRPS);
    }
    
    /**
     * Stops the intake rollers.
     * Sets motor output to neutral and state to STOWED.
     */
    public void stop() {
        currentState = IntakeState.STOWED;
        io.stop();
    }
    
    /**
     * Sets the voltage output for the intake (open-loop control).
     * Primarily used for characterization and testing.
     * 
     * @param volts Voltage to apply (-12 to +12V)
     */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
    
    /**
     * Gets the current state of the intake.
     * 
     * @return Current IntakeState
     */
    public IntakeState getCurrentState() {
        return currentState;
    }
    
    /**
     * Checks if the intake rollers are at their target velocity within tolerance.
     * 
     * @return true if rollers are within tolerance of target
     */
    public boolean isAtTargetVelocity() {
        double error = Math.abs(inputs.velocityRPS - inputs.targetVelocityRPS);
        return error < IntakeConstants.VELOCITY_TOLERANCE;
    }
    
    /**
     * Gets the velocity error for the intake rollers.
     * 
     * @return Velocity error in rotations per second
     */
    public double getVelocityError() {
        return inputs.targetVelocityRPS - inputs.velocityRPS;
    }
    
    /**
     * Gets the current velocity of the intake rollers.
     * 
     * @return Current velocity in rotations per second
     */
    public double getVelocity() {
        return inputs.velocityRPS;
    }
    
    /**
     * Gets the target velocity of the intake rollers.
     * 
     * @return Target velocity in rotations per second
     */
    public double getTargetVelocity() {
        return inputs.targetVelocityRPS;
    }
}