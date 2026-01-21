package frc.robot.subsystems.Climberz;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Climberz.ClimberzIO.ClimberzIOInputs;

public class ClimberzBase extends SubsystemBase {
    
    public enum ClimberState {
        STOPPED,
        CLIMBING_UP,
        CLIMBING_DOWN
    }
    
    private final ClimberzIO io;
    private final ClimberzIOInputs inputs = new ClimberzIOInputs();
    
    private ClimberState currentState = ClimberState.STOPPED;
    
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
        Logger.recordOutput("Climberz/LeaderCurrentAmps", inputs.leaderCurrentAmps);
        Logger.recordOutput("Climberz/FollowerCurrentAmps", inputs.followerCurrentAmps);
        Logger.recordOutput("Climberz/TotalCurrentAmps", inputs.leaderCurrentAmps + inputs.followerCurrentAmps);
        Logger.recordOutput("Climberz/LeaderTempCelsius", inputs.leaderTempCelsius);
        Logger.recordOutput("Climberz/FollowerTempCelsius", inputs.followerTempCelsius);
        Logger.recordOutput("Climberz/State", currentState.toString());
    }
    
    // ==================== BASIC CONTROL ====================
    
    /**
     * Run climber up (pull robot up)
     */
    public void climbUp() {
        currentState = ClimberState.CLIMBING_UP;
        io.setDutyCycle(ClimberzConstants.climbUpDutyCycle);
    }
    
    /**
     * Run climber down (extend arm)
     */
    public void climbDown() {
        currentState = ClimberState.CLIMBING_DOWN;
        io.setDutyCycle(ClimberzConstants.climbDownDutyCycle);
    }
    
    /**
     * Stop the climber
     */
    public void stop() {
        currentState = ClimberState.STOPPED;
        io.stop();
    }
    
    /**
     * Run climber at specified duty cycle
     */
    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }
    
    // ==================== GETTERS ====================
    
    public ClimberState getState() {
        return currentState;
    }
    
    public double getPosition() {
        return inputs.positionRotations;
    }
    
    public double getTotalCurrent() {
        return inputs.leaderCurrentAmps + inputs.followerCurrentAmps;
    }
    
}