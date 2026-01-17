package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Index.IndexBase;
import frc.robot.subsystems.Index.IndexBase.IndexState;
import frc.robot.subsystems.Intake.IntakeBase;

/**
 * Command that coordinates intake and index during intaking.
 * - Deploys intake and runs rollers
 * - Runs index until a ball is detected by CANRange
 * - Keeps intake running until command is interrupted or crash detected
 * - Index stops when ball is staged, intake continues
 */
public class IntakeIndexCommand extends Command {
    
    private final IntakeBase intake; 
    private final IndexBase index;   
    private boolean ballWasDetected = false;
    
    public IntakeIndexCommand(IntakeBase intake, IndexBase index) {
        this.intake = intake;
        this.index = index;
        
        // Require both subsystems
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        ballWasDetected = false;
        
        // Deploy intake (this also runs rollers)
        intake.deploy();
        
        // Start indexing
        index.requestIntakeIndex(true);
    }
    
    @Override
    public void execute() {
        // Check if ball is now detected
        if (index.hasGamePiece() && !ballWasDetected) {
            // Ball just detected - stop index but keep intake running
            ballWasDetected = true;
            index.stop();
        }
        
        // If ball was detected but is now gone (shot was fired), resume indexing
        if (ballWasDetected 
            && !index.hasGamePiece() 
            && index.getState() != IndexState.INDEXING 
            && index.getState() != IndexState.REVERSING) {
            ballWasDetected = false;
            index.index();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop both subsystems
        intake.retract();
        index.requestIntakeIndex(false);
    }
    
    @Override
    public boolean isFinished() {
        // Run until interrupted by driver
        // Crash detection is handled internally by IntakeBase
        return false;
    }
    
    @Override
    public String getName() {
        return "IntakeIndexCommand";
    }
}

