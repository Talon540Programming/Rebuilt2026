package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.Pivot.PivotIO;
import frc.robot.subsystems.Intake.Pivot.PivotIO.PivotIOInputs;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIO.RollerIOInputs;

public class IntakeBase extends SubsystemBase {
    
    public enum IntakeState {
        STOWED,
        DEPLOYING,
        DEPLOYED,
        RETRACTING
    }
    
    private final PivotIO pivotIO;
    private final RollerIO rollerIO;
    private double deployTimer = 0.0;
    private static final double DEPLOY_TIME_SECONDS = 0.5;
    private double retractTimer = 0.0;
    private static final double RETRACT_TIME_SECONDS = 0.5;
    
    // Use the base inputs class instead of AutoLogged
    private final PivotIOInputs pivotInputs = new PivotIOInputs();
    private final RollerIOInputs rollerInputs = new RollerIOInputs();
    
    private IntakeState currentState = IntakeState.STOWED;
    private boolean intakeEnabled = false;
    
    public IntakeBase(PivotIO pivotIO, RollerIO rollerIO) {
        this.pivotIO = pivotIO;
        this.rollerIO = rollerIO;
    }
    
    @Override
    public void periodic() {
        // Update inputs from IO layers
        pivotIO.updateInputs(pivotInputs);
        rollerIO.updateInputs(rollerInputs);

        // Collision detection using velocity - cleaner than position delta
        if (intakeEnabled) {
            // Velocity negative = moving backward, high current = motor fighting something
            boolean beingBackdriven = pivotInputs.velocityRotPerSec < -0.5 
                                    && pivotInputs.currentAmps > IntakeConstants.kCollisionCurrentThreshold;
            
            if (beingBackdriven) {
                handleCollision();
            }
        }

        if (currentState == IntakeState.DEPLOYING) {
            deployTimer += 0.02;

            if (deployTimer >= DEPLOY_TIME_SECONDS) {
                currentState = IntakeState.DEPLOYED;
                pivotIO.stop();
                pivotIO.setBrakeMode(true);
            }

        if (currentState == IntakeState.RETRACTING) {
            retractTimer += 0.02;
            
            if (retractTimer >= RETRACT_TIME_SECONDS) {
                currentState = IntakeState.STOWED;
                pivotIO.stop();
                pivotIO.setBrakeMode(true);
            }
        }
    }

        // Log inputs manually
        Logger.recordOutput("Intake/Pivot/PositionRotations", pivotInputs.positionRotations);
        Logger.recordOutput("Intake/Pivot/VelocityRotPerSec", pivotInputs.velocityRotPerSec);
        Logger.recordOutput("Intake/Pivot/AppliedVolts", pivotInputs.appliedVolts);
        Logger.recordOutput("Intake/Pivot/CurrentAmps", pivotInputs.currentAmps);
        Logger.recordOutput("Intake/Pivot/TempCelsius", pivotInputs.tempCelsius);
        
        Logger.recordOutput("Intake/Roller/VelocityRotPerSec", rollerInputs.velocityRotPerSec);
        Logger.recordOutput("Intake/Roller/AppliedVolts", rollerInputs.appliedVolts);
        Logger.recordOutput("Intake/Roller/CurrentAmps", rollerInputs.currentAmps);
        Logger.recordOutput("Intake/Roller/TempCelsius", rollerInputs.tempCelsius);
        Logger.recordOutput("Intake/Roller/HasGamePiece", rollerInputs.hasGamePiece);
        
        // Log state
        Logger.recordOutput("Intake/State", currentState.toString());
        Logger.recordOutput("Intake/Enabled", intakeEnabled);
        Logger.recordOutput("Intake/HasGamePiece", hasGamePiece());
    }
    
    /**
     * Deploy the intake and run rollers
     */
   public void deploy() {
    intakeEnabled = true;
    currentState = IntakeState.DEPLOYING;
    deployTimer = 0.0;
    pivotIO.setBrakeMode(true);  // Brake mode during motion is fine
    pivotIO.setDutyCycle(IntakeConstants.kDeployDutyCycle);
    rollerIO.setDutyCycle(IntakeConstants.kRollerIntakeDutyCycle);
}

    /**
 * Handle detected collision - retract intake to protect mechanism
 */
private void handleCollision() {
    Logger.recordOutput("Intake/CollisionDetected", true);
    retract();  // Just retract - driver can re-deploy when safe
}
    
    /**
     * Retract the intake and stop rollers
     */
    public void retract() {
        intakeEnabled = false;
        currentState = IntakeState.RETRACTING;
        retractTimer = 0.0;
    }
    
    /**
     * Toggle intake state
     */
    public void toggle() {
        if (intakeEnabled) {
            retract();
        } else {
            deploy();
        }
    }
    
    /**
     * Stop all intake motors
     */
    public void stop() {
        intakeEnabled = false;
        currentState = IntakeState.STOWED;
        deployTimer = 0.0;  
        retractTimer = 0.0;  
        pivotIO.stop();
        rollerIO.stop();
    }
    
    /**
     * Run rollers in reverse to eject game piece
     */
    public void eject() {
        rollerIO.setDutyCycle(IntakeConstants.kRollerEjectDutyCycle);
    }
    
    /**
     * Check if a game piece is detected via current spike
     */
    public boolean hasGamePiece() {
        return rollerInputs.hasGamePiece;
    }
    
    /**
     * Check if intake is deployed
     */
    public boolean isDeployed() {
        return intakeEnabled;
    }
    
    /**
     * Get current state
     */
    public IntakeState getState() {
        return currentState;
    }
    
    /**
     * Get roller current for external monitoring
     */
    public double getRollerCurrent() {
        return rollerInputs.currentAmps;
    }
    
    // ==================== COMMANDS ====================
    
    /**
     * Command to deploy intake (runs until interrupted)
     */
    public Command deployCommand() {
        return runOnce(this::deploy)
            .andThen(run(() -> {})) // Keep running
            .finallyDo((interrupted) -> retract())
            .withName("Intake Deploy");
    }
    
    /**
     * Command to toggle intake state
     */
    public Command toggleCommand() {
        return runOnce(this::toggle).withName("Intake Toggle");
    }
    
    /**
     * Command to eject game piece
     */
    public Command ejectCommand() {
        return runOnce(this::eject)
            .andThen(run(() -> {}))
            .finallyDo((interrupted) -> stop())
            .withName("Intake Eject");
    }
}