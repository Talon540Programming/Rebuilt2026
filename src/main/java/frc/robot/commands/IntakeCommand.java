package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeBase;
import frc.robot.subsystems.Intake.IntakeConstants;

/**
 * Command that coordinates intake and index during intaking.
 * - Deploys intake and runs rollers
 * - Runs index until a ball is detected by CANRange
 * - Keeps intake running until command is interrupted or crash detected
 * - Index stops when ball is staged, intake continues
 */
public class IntakeCommand extends Command {
    
    private final IntakeBase intake;
    private double deployStartTime;
    private boolean rollersStarted;
    
    public IntakeCommand(IntakeBase intake) {
        this.intake = intake;
        
        addRequirements(intake);
    }
    
    @Override
    public void initialize() { 
        intake.deploy();
        deployStartTime = Timer.getFPGATimestamp();
        rollersStarted = false;
    }
    
    @Override
    public void execute() {
        if (!rollersStarted && 
            Timer.getFPGATimestamp() - deployStartTime >= IntakeConstants.rollerStartDelaySeconds) {
            intake.startRollers();
            rollersStarted = true;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        intake.retract();
    }
    
    @Override
    public boolean isFinished() {
        // Run until interrupted by driver
        // Crash detection is handled internally by IntakeBase
        return false;
    }
    
    @Override
    public String getName() {
        return "IntakeCommand";
    }
}

