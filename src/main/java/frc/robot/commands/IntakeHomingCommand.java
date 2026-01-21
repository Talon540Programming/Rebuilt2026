package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeBase;
import frc.robot.subsystems.Intake.IntakeConstants;

/**
 * Command to home the intake extension by running toward the hard stop until stalled,
 * then zeroing the encoder at the stowed position.
 */
public class IntakeHomingCommand extends Command {
    
    private final IntakeBase intake;
    
    private enum HomingState {
        RESETTING,
        MOVING_TO_STOP,
        ZEROING,
        FINISHED
    }
    
    private HomingState state = HomingState.RESETTING;
    
    public IntakeHomingCommand(IntakeBase intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        state = HomingState.RESETTING;
        intake.resetHomingState();
    }
    
    @Override
    public void execute() {
        switch (state) {
            case RESETTING:
                state = HomingState.MOVING_TO_STOP;
                break;
                
            case MOVING_TO_STOP:
                intake.setExtensionDutyCycle(IntakeConstants.extensionHomingDutyCycle);
                
                if (intake.isExtensionStalled()) {
                    state = HomingState.ZEROING;
                }
                break;
                
            case ZEROING:
                intake.zeroExtensionAtStowed();
                state = HomingState.FINISHED;
                break;
                
            case FINISHED:
                break;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        intake.stopExtension();
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