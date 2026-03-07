package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeBase;
import frc.robot.subsystems.Intake.IntakeBase.Goal;
import frc.robot.subsystems.Intake.IntakeConstants;

/**
 * Command that "jiggles" the intake to dislodge stuck fuel.
 * Cycles between half extension and full extension while held.
 * Restores previous state when released.
 */
public class JiggleCommand extends Command {
    
    private enum JiggleState {
        RETRACTING_TO_HALF,
        EXTENDING_TO_FULL
    }
    
    private final IntakeBase intake;
    private Goal previousGoal;
    private boolean rollersWereRunning;
    private JiggleState jiggleState;
    private double halfPosition;
    private double fullPosition;
    
    public JiggleCommand(IntakeBase intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        // Save previous state
        previousGoal = intake.getGoal();
        rollersWereRunning = intake.getRollerVolts() > 0;
        
        // Calculate positions
        double stowed = IntakeConstants.extensionStowedPosRot.get();
        double deployed = IntakeConstants.extensionDeployedPosRot.get();
        halfPosition = stowed + (deployed - stowed) / 2.0;
        fullPosition = deployed;
        
        // Stop rollers during jiggle
        intake.stopRollers();
        
        // Start by retracting to half
        jiggleState = JiggleState.RETRACTING_TO_HALF;
        intake.setGoalPosition(halfPosition);
    }
    
    @Override
    public void execute() {
        switch (jiggleState) {
            case RETRACTING_TO_HALF:
                if (intake.isAtPosition(halfPosition)) {
                    // Reached half, now extend to full
                    jiggleState = JiggleState.EXTENDING_TO_FULL;
                    intake.setGoalPosition(fullPosition);
                }
                break;
                
            case EXTENDING_TO_FULL:
                if (intake.isAtPosition(fullPosition)) {
                    // Reached full, now retract to half again
                    jiggleState = JiggleState.RETRACTING_TO_HALF;
                    intake.setGoalPosition(halfPosition);
                }
                break;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Restore previous goal state using setGoal to properly update state machine
        intake.setGoal(previousGoal);
        
        // Restore rollers if they were running
        if (rollersWereRunning) {
            intake.startRollers();
        }
    }
    
    @Override
    public boolean isFinished() {
        // Run until interrupted (button released)
        return false;
    }
}