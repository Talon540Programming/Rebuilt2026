package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.PivotIO.PivotIOInputs;
import frc.robot.subsystems.Intake.RollerIO.RollerIOInputs;

public class IntakeBase extends SubsystemBase {
    
    public enum IntakeState {
        STOWED,
        DEPLOYING,
        DEPLOYED,
        RETRACTING
    }
    
    private final PivotIO pivotIO;
    private final RollerIO rollerIO;
    
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
        pivotIO.setDutyCycle(IntakeConstants.kDeployDutyCycle);
        rollerIO.setDutyCycle(IntakeConstants.kRollerIntakeDutyCycle);
    }
    
    /**
     * Retract the intake and stop rollers
     */
    public void retract() {
        intakeEnabled = false;
        currentState = IntakeState.RETRACTING;
        pivotIO.setDutyCycle(IntakeConstants.kRetractDutyCycle);
        rollerIO.stop();
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