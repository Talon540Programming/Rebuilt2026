package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterBase;
import frc.robot.subsystems.Shooter.ShooterConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Command to home the hood by running toward the hard stop for a fixed duration,
 * then stopping the motor and zeroing the encoder at the minimum position.
 */
public class HoodHomingCommand extends Command {
    
    private final ShooterBase shooter;
    
    private enum HomingState {
        MOVING_TO_STOP,
        STOPPING,
        ZEROING,
        FINISHED
    }
    
    private HomingState state = HomingState.MOVING_TO_STOP;
    private double startTime;
    
    public HoodHomingCommand(ShooterBase shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        state = HomingState.MOVING_TO_STOP;
        shooter.resetHomingState();
        startTime = Timer.getFPGATimestamp();
        
        // Start moving toward hard stop
        shooter.setHoodDutyCycle(ShooterConstants.hoodHomingDutyCycle.get());
        
        Logger.recordOutput("Shooter/Hood/HomingState", state.toString());
    }
    
    @Override
    public void execute() {
        double elapsedTime = Timer.getFPGATimestamp() - startTime;
        
        Logger.recordOutput("Shooter/Hood/HomingElapsedTime", elapsedTime);
        Logger.recordOutput("Shooter/Hood/HomingState", state.toString());
        
        switch (state) {
            case MOVING_TO_STOP:
                // Check if timeout reached
                if (elapsedTime >= ShooterConstants.hoodHomingTimeoutSeconds.get()) {
                    shooter.stopHood();
                    state = HomingState.STOPPING;
                }
                break;
                
            case STOPPING:
                // Wait one cycle for motor to fully stop before zeroing
                state = HomingState.ZEROING;
                break;
                
            case ZEROING:
                // Zero encoder at hard stop
                shooter.zeroHoodAtMin();
                state = HomingState.FINISHED;
                Logger.recordOutput("Shooter/Hood/HomingComplete", true);
                break;
                
            case FINISHED:
                // Do nothing, isFinished will return true
                break;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        shooter.stopHood();
        shooter.exitHomingState();
        
        if (interrupted) {
            Logger.recordOutput("Shooter/Hood/HomingInterrupted", true);
        }
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