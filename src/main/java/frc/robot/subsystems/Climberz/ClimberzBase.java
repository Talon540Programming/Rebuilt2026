package frc.robot.subsystems.Climberz;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.ClimberHomingCommand;
import frc.robot.subsystems.Climberz.ClimberzIO.ClimberzIOInputs;

public class ClimberzBase extends SubsystemBase {
    
    public enum ClimberState {
        STOPPED,
        CLIMBING_UP,
        CLIMBING_DOWN,
        POSITION_CONTROL,
        HOMING
    }
    
    private final ClimberzIO io;
    private final ClimberzIOInputs inputs = new ClimberzIOInputs();
    
    private ClimberState currentState = ClimberState.STOPPED;
    private boolean homed = false;
    private double targetPositionRot = 0.0;
    
    public ClimberzBase(ClimberzIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        
        // Manual logging
        Logger.recordOutput("Climberz/PositionRotations", inputs.positionRotations);
        Logger.recordOutput("Climberz/VelocityRotPerSec", inputs.velocityRotPerSec);
        Logger.recordOutput("Climberz/AppliedVolts", inputs.appliedVolts);
        Logger.recordOutput("Climberz/CurrentAmps", inputs.currentAmps);
        Logger.recordOutput("Climberz/TempCelsius", inputs.tempCelsius);
        Logger.recordOutput("Climberz/State", currentState.toString());
        Logger.recordOutput("Climberz/Homed", homed);
        Logger.recordOutput("Climberz/TargetPositionRot", targetPositionRot);
    }
    
    // ==================== DUTY CYCLE CONTROL (BACKUP) ====================
    
    /**
     * Run climber up (pull robot up) - brake mode + positive duty cycle
     */
    public void climbUp() {
        currentState = ClimberState.CLIMBING_UP;
        io.setBrakeMode(true);
        io.setDutyCycle(ClimberzConstants.climbUpDutyCycle);
    }
    
    /**
     * Release climber (let springs extend) - coast mode
     */
    public void climbDown() {
        currentState = ClimberState.CLIMBING_DOWN;
        io.setBrakeMode(false);
        io.setDutyCycle(ClimberzConstants.climbDownDutyCycle);
    }
    
    /**
     * Stop the climber - brake mode to hold position
     */
    public void stop() {
        currentState = ClimberState.STOPPED;
        io.setBrakeMode(true);
        io.stop();
    }
    
    // ==================== POSITION CONTROL ====================
    
    /**
     * Move climber to retracted position using Motion Magic
     */
    public void goToRetracted() {
        if (!homed) {
            Logger.recordOutput("Climberz/Warning", "Not homed - ignoring goToRetracted");
            return;
        }
        currentState = ClimberState.POSITION_CONTROL;
        targetPositionRot = ClimberzConstants.retractedPositionRot.get();
        io.setBrakeMode(true);
        io.runMotionMagicPosition(targetPositionRot);
    }
    
    /**
     * Extend climber using coast mode (let springs do the work)
     */
    public void goToExtended() {
        currentState = ClimberState.CLIMBING_DOWN;
        io.setBrakeMode(false);
        io.setDutyCycle(ClimberzConstants.climbDownDutyCycle);
    }
    
    /**
     * Check if climber is at target position
     */
    public boolean isAtTarget() {
        return Math.abs(inputs.positionRotations - targetPositionRot) 
            < ClimberzConstants.positionToleranceRot.get();
    }
    
    // ==================== HOMING ====================
    
    /**
     * Check if climber is stalled (for homing detection)
     */
    public boolean isStalled() {
        return Math.abs(inputs.velocityRotPerSec) < ClimberzConstants.homingVelThreshold.get()
            && Math.abs(inputs.currentAmps) > ClimberzConstants.homingCurrentThreshold.get();
    }
    
    /**
     * Set duty cycle directly (for homing)
     */
    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }
    
    /**
     * Zero the encoder at current position
     */
    public void zeroAtRetracted() {
        io.setPosition(ClimberzConstants.retractedPositionRot.get());
        homed = true;
        Logger.recordOutput("Climberz/Homed", true);
    }
    
    /**
     * Reset homing state
     */
    public void resetHomingState() {
        homed = false;
        currentState = ClimberState.HOMING;
    }
    
    /**
     * Exit homing state
     */
    public void exitHomingState() {
        if (currentState == ClimberState.HOMING) {
            currentState = ClimberState.STOPPED;
        }
    }
    
    /**
     * Get the homing command
     */
    public Command homingSequence() {
        return new ClimberHomingCommand(this).withTimeout(3.0);
    }
    
    /**
     * Check if climber is homed
     */
    public boolean isHomed() {
        return homed;
    }
    
    // ==================== GETTERS ====================
    
    public ClimberState getState() {
        return currentState;
    }
    
    public double getPosition() {
        return inputs.positionRotations;
    }
    
    public double getCurrent() {
        return inputs.currentAmps;
    }
    
    public double getVelocity() {
        return inputs.velocityRotPerSec;
    }
    
    /**
     * Retract the climber (for homing or general use) - same as climbUp
     */
    public void retract() {
        climbUp();
    }
}