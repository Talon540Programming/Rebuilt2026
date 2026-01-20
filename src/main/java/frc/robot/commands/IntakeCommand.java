package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeBase;

/**
 * Command that coordinates intake and index during intaking.
 * - Deploys intake and runs rollers
 * - Runs index until a ball is detected by CANRange
 * - Keeps intake running until command is interrupted or crash detected
 * - Index stops when ball is staged, intake continues
 */
public class IntakeCommand extends Command {
    
    private final IntakeBase intake;    
    
    public IntakeCommand(IntakeBase intake) {
        this.intake = intake;
        
        addRequirements(intake);
    }
    
    @Override
    public void initialize() { 
        // Deploy intake (this also runs rollers)
        intake.deploy();
    }
    
    @Override
    public void execute() {}
    
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

