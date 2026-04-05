package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climberz.ClimberzBase;
import frc.robot.subsystems.Climberz.ClimberzConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Command to home the climber by running toward the retracted position until stalled,
 * then zeroing the encoder.
 * 
 * Uses hybrid approach: stall detection + timeout backup.
 */
public class ClimberHomingCommand extends Command {
    
    private final ClimberzBase climber;
    
    private enum HomingState {
        MOVING_TO_STOP,
        STOPPING,
        ZEROING,
        FINISHED
    }
    
    private HomingState state = HomingState.MOVING_TO_STOP;
    private double startTime;
    
    public ClimberHomingCommand(ClimberzBase climber) {
        this.climber = climber;
        addRequirements(climber);
    }
    
    @Override
    public void initialize() {
        state = HomingState.MOVING_TO_STOP;
        climber.resetHomingState();
        startTime = Timer.getFPGATimestamp();
        
        // Start moving toward retracted position (positive = retract/wind)
        climber.setDutyCycle(ClimberzConstants.homingDutyCycle);
        
        Logger.recordOutput("Climberz/HomingState", state.toString());
    }
    
    @Override
    public void execute() {
        double elapsedTime = Timer.getFPGATimestamp() - startTime;
        
        Logger.recordOutput("Climberz/HomingElapsedTime", elapsedTime);
        Logger.recordOutput("Climberz/HomingState", state.toString());
        
        switch (state) {
            case MOVING_TO_STOP:
                // Check for stall (hit the hard stop)
                if (climber.isStalled()) {
                    climber.stop();
                    state = HomingState.STOPPING;
                    Logger.recordOutput("Climberz/HomingStopReason", "Stall");
                }
                
                // Check timeout
                if (elapsedTime >= ClimberzConstants.homingTimeoutSeconds.get()) {
                    climber.stop();
                    state = HomingState.STOPPING;
                    Logger.recordOutput("Climberz/HomingStopReason", "Timeout");
                }
                break;
                
            case STOPPING:
                // Wait one cycle for motor to fully stop before zeroing
                state = HomingState.ZEROING;
                break;
                
            case ZEROING:
                // Zero encoder at retracted position
                climber.zeroAtRetracted();
                state = HomingState.FINISHED;
                Logger.recordOutput("Climberz/HomingComplete", true);
                break;
                
            case FINISHED:
                // Do nothing, isFinished will return true
                break;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        climber.stop();
        climber.exitHomingState();
        
        if (interrupted) {
            Logger.recordOutput("Climberz/HomingInterrupted", true);
        }
    }
    
    @Override
    public boolean isFinished() {
        return state == HomingState.FINISHED;
    }
    
    @Override
    public String getName() {
        return "ClimberHomingCommand";
    }
}