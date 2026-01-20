package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterBase;
import frc.robot.subsystems.Shooter.ShooterConstants;

/**
 * Command to home the hood by running toward the hard stop until stalled,
 * then zeroing the encoder at the minimum position.
 */
public class HoodHomingCommand extends Command {
    
    private final ShooterBase shooter;
    
    private enum HomingState {
        RESETTING,
        MOVING_TO_STOP,
        ZEROING,
        FINISHED
    }
    
    private HomingState state = HomingState.RESETTING;
    
    public HoodHomingCommand(ShooterBase shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        state = HomingState.RESETTING;
        shooter.resetHomingState();
    }
    
    @Override
    public void execute() {
        switch (state) {
            case RESETTING:
                // Move to next state immediately
                state = HomingState.MOVING_TO_STOP;
                break;
                
            case MOVING_TO_STOP:
                // Run toward hard stop
                shooter.setHoodDutyCycle(ShooterConstants.hoodHomingDutyCycle.get());
                
                // Check if stalled
                if (shooter.isHoodStalled()) {
                    state = HomingState.ZEROING;
                }
                break;
                
            case ZEROING:
                // Zero encoder at hard stop
                shooter.zeroHoodAtMin();
                state = HomingState.FINISHED;
                break;
                
            case FINISHED:
                // Do nothing, isFinished will return true
                break;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopHood();
    }
    
    @Override
    public boolean isFinished() {
        return state == HomingState.FINISHED;
    }
    
    @Override
    public String getName() {
        return "HoodHomingCommand";
    }
}