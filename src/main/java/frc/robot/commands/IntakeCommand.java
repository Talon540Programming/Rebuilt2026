package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeBase;

/**
 * Command that coordinates intake and index during intaking.
 * - Deploys intake and runs rollers immediately
 * - Keeps rollers running during extension and retraction
 * - Rollers stop automatically when fully stowed (handled by IntakeBase)
 */
public class IntakeCommand extends Command {
    
    private final IntakeBase intake;
    
    public IntakeCommand(IntakeBase intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() { 
        intake.deploy();
        intake.startRollers();
    }
    
    @Override
    public void execute() {
        // Nothing needed - rollers running, extension controlled by periodic()
    }
    
    @Override
    public void end(boolean interrupted) {
        intake.retract();
        // Don't stop rollers here - they'll stop automatically when stowed
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public String getName() {
        return "IntakeCommand";
    }
}