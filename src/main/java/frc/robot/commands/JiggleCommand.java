package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeBase;
import frc.robot.subsystems.Intake.IntakeBase.Goal;
import frc.robot.subsystems.Intake.IntakeConstants;

/**
 * Command that "jiggles" the intake to dislodge stuck fuel.
 * Continuously cycles between retracted and deployed positions.
 * Stops rollers during jiggle and restores previous state on end.
 */
public class JiggleCommand extends Command {
    
    private enum JiggleState {
        RETRACTING,
        DEPLOYING
    }
    
    private final IntakeBase intake;
    private Goal previousGoal;
    private boolean rollersWereRunning;
    private JiggleState jiggleState;
    
    public JiggleCommand(IntakeBase intake) {
        this.intake = intake;
        addRequirements(intake);
    }
    
    @Override
    public void initialize() {
        // Save previous state
        previousGoal = intake.getGoal();
        rollersWereRunning = intake.getRollerVolts() > 0;
        
        // Stop rollers during jiggle
        intake.stopRollers();
        
        // Start by retracting
        jiggleState = JiggleState.RETRACTING;
        intake.setGoalPosition(IntakeConstants.extensionStowedPosRot.get());
    }
    
    @Override
    public void execute() {
        switch (jiggleState) {
            case RETRACTING:
                if (intake.isAtStowed()) {
                    // Reached stowed, now deploy
                    jiggleState = JiggleState.DEPLOYING;
                    intake.setGoalPosition(IntakeConstants.extensionDeployedPosRot.get()/2);
                }
                break;
                
            case DEPLOYING:
                if (intake.isAtDeployed()) {
                    // Reached deployed, now retract again
                    jiggleState = JiggleState.RETRACTING;
                    intake.setGoalPosition(IntakeConstants.extensionStowedPosRot.get());
                }
                break;
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Restore previous goal state
        if (previousGoal == Goal.DEPLOYED) {
            intake.deploy();
        } else {
            intake.retract();
        }
        
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