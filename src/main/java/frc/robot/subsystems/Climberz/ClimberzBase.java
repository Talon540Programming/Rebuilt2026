package frc.robot.subsystems.Climberz;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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
        Logger.recordOutput("Climberz/AppliedVolts", inputs.appliedVolts);
        Logger.recordOutput("Climberz/CurrentAmps", inputs.currentAmps);
        Logger.recordOutput("Climberz/TempCelsius", inputs.tempCelsius);
        Logger.recordOutput("Climberz/State", currentState.toString());
        
        // Only log position/velocity in simulation (no encoder feedback in real life)
        if (Robot.isSimulation()) {
            Logger.recordOutput("Climberz/Sim/PositionRotations", inputs.positionRotations);
            Logger.recordOutput("Climberz/Sim/VelocityRotPerSec", inputs.velocityRotPerSec);
        }
    }
    
    // ==================== BASIC CONTROL ====================
    
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
    
    /**
     * Retract the climber (for homing or general use) - same as climbUp
     */
    public void retract() {
        climbUp();
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
    
}