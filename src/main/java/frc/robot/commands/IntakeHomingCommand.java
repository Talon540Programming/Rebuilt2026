package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeBase;
import frc.robot.subsystems.Intake.IntakeConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Command to home the intake extension by running toward the hard stop for a fixed duration,
 * then stopping the motor and zeroing the encoder at the stowed position.
 */
public class IntakeHomingCommand extends Command {
    
    private final IntakeBase intake;
    
    private enum HomingState {
        MOVING_TO_STOP,
        STOPPING,
        ZEROING,
        FINISHED
    }
    
    private HomingState state = HomingState.MOVING_TO_STOP;
    private double startTime;
    
    public IntakeHomingCommand(IntakeBase intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        state = HomingState.MOVING_TO_STOP;
        intake.resetHomingState();
        startTime = Timer.getFPGATimestamp();
        
        // Start moving toward hard stop
        intake.setExtensionDutyCycle(IntakeConstants.extensionHomingDutyCycle);
        
        Logger.recordOutput("Intake/Extension/HomingState", state.toString());
    }
    
    @Override
    public void execute() {
        double elapsedTime = Timer.getFPGATimestamp() - startTime;
        
        Logger.recordOutput("Intake/Extension/HomingElapsedTime", elapsedTime);
        Logger.recordOutput("Intake/Extension/HomingState", state.toString());
        
        switch (state) {
            case MOVING_TO_STOP:
                // Check if timeout reached
                if (elapsedTime >= IntakeConstants.extensionHomingTimeoutSeconds) {
                    intake.stopExtension();
                    state = HomingState.STOPPING;
                }
                break;
                
            case STOPPING:
                // Wait one cycle for motor to fully stop before zeroing
                state = HomingState.ZEROING;
                break;
                
            case ZEROING:
                // Zero encoder at hard stop
                intake.zeroExtensionAtStowed();
                state = HomingState.FINISHED;
                Logger.recordOutput("Intake/Extension/HomingComplete", true);
                break;
                
            case FINISHED:
                // Do nothing, isFinished will return true
                break;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        intake.stopExtension();
        
        if (interrupted) {
            Logger.recordOutput("Intake/Extension/HomingInterrupted", true);
        }
    }
    
    @Override
    public boolean isFinished() {
        return state == HomingState.FINISHED;
    }
    
    @Override
    public String getName() {
        return "IntakeHomingCommand";
    }
}