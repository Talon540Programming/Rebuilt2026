package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeHomingCommand;
import frc.robot.subsystems.Intake.Extension.ExtensionIO;
import frc.robot.subsystems.Intake.Extension.ExtensionIO.ExtensionIOInputs;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIO.RollerIOInputs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;



public class IntakeBase extends SubsystemBase {
    
    public enum IntakeState {
        STOWED,
        DEPLOYING,
        DEPLOYED,
        RETRACTING
    }

    public enum Goal { 
        STOWED, 
        DEPLOYED 
    }

    private Debouncer crashDebouncer =
        new Debouncer(IntakeConstants.extensionCrashDebounceSecs.get(), DebounceType.kRising);

    private Goal goal = Goal.STOWED;
    private double goalPosRot = IntakeConstants.extensionStowedPosRot.get();

    
    private final ExtensionIO extensionIO;
    private final RollerIO rollerIO;
    
    // Use the base inputs class instead of AutoLogged
    private final ExtensionIOInputs extensionInputs = new ExtensionIOInputs();
    private final RollerIOInputs rollerInputs = new RollerIOInputs();
    
    private IntakeState currentState = IntakeState.STOWED;
    private boolean intakeEnabled = false;

    private boolean homed = false;

    // Track goal changes so crash detection doesn’t trip during normal motion
    private double goalChangeTimestampSec = 0.0;

    // If you want: latch a crash for driver visibility
    private boolean crashLatched = false;

    
    public IntakeBase(ExtensionIO extensionIO, RollerIO rollerIO) {
        this.extensionIO = extensionIO;
        this.rollerIO = rollerIO;
        crashDebouncer = new Debouncer(IntakeConstants.extensionCrashDebounceSecs.get(), DebounceType.kRising);
    }
    
    @Override
    public void periodic() {
        // Update inputs from IO layers
        extensionIO.updateInputs(extensionInputs);
        rollerIO.updateInputs(rollerInputs);

        if (DriverStation.isEnabled() && homed) {
            extensionIO.runMotionMagicPosition(goalPosRot, 0.0);
            } 
            else if (!DriverStation.isEnabled()) {
                extensionIO.stop();
        }

        final double errorRot = goalPosRot - extensionInputs.positionRotations;
        final boolean atPivotGoal =
            Math.abs(errorRot) < IntakeConstants.extensionAllowedErrorRot.get()
            && Math.abs(extensionInputs.velocityRotPerSec) < IntakeConstants.extensionAllowedVelRotPerSec.get();
   
       // Update state machine based on goal + atGoal
        if (goal == Goal.DEPLOYED) {
            currentState = atPivotGoal ? IntakeState.DEPLOYED : IntakeState.DEPLOYING;
        } 
        else {
            currentState = atPivotGoal ? IntakeState.STOWED : IntakeState.RETRACTING;
            
            // Stop rollers automatically when fully stowed
            if (atPivotGoal && currentState == IntakeState.STOWED) {
                rollerIO.stop();
            }
        }

        final double nowSec = Timer.getFPGATimestamp();
        final boolean settledAfterGoalChange =
            (nowSec - goalChangeTimestampSec) > IntakeConstants.extensionCrashIgnoreAfterGoalChangeSecs.get();

        final boolean crashCondition =
            homed
            && DriverStation.isEnabled()
            && goal == Goal.DEPLOYED
            && settledAfterGoalChange
            && Math.abs(errorRot) > IntakeConstants.extensionCrashMinErrorRot.get()
            && Math.abs(extensionInputs.velocityRotPerSec) < IntakeConstants.extensionCrashMaxVelRotPerSec.get()
            && extensionInputs.currentAmps > IntakeConstants.extensionCrashCurrentAmps.get();

        boolean crashed = crashDebouncer.calculate(crashCondition);

       
        if (crashed && !crashLatched) {
            crashLatched = true;
            Logger.recordOutput("Intake/CrashDetected", true);

            // Auto-stow to protect mechanism
            retract();
        } else {
            Logger.recordOutput("Intake/CrashDetected", false);
        }

        Logger.recordOutput("Intake/Extension/ErrorRot", errorRot);
        Logger.recordOutput("Intake/Extension/AtGoal", atPivotGoal);
        Logger.recordOutput("Intake/CrashLatched", crashLatched);

        // Log inputs manually
        Logger.recordOutput("Intake/Extension/PositionRotations", extensionInputs.positionRotations);
        Logger.recordOutput("Intake/Extension/VelocityRotPerSec", extensionInputs.velocityRotPerSec);
        Logger.recordOutput("Intake/Extension/AppliedVolts", extensionInputs.appliedVolts);
        Logger.recordOutput("Intake/Extension/CurrentAmps", extensionInputs.currentAmps);
        Logger.recordOutput("Intake/Extension/TempCelsius", extensionInputs.tempCelsius);
        
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
     * Deploy the intake (extension only, no rollers)
     */
    public void deploy() {
        intakeEnabled = true;
        setGoal(Goal.DEPLOYED);
        currentState = IntakeState.DEPLOYING;
    }
    
    /**
     * Start the intake rollers
     */
    public void startRollers() {
        rollerIO.setDutyCycle(IntakeConstants.rollerIntakeDutyCycle);
    }

    public void stopRollers(){
        rollerIO.setDutyCycle(0);
    }

    
    /**
     * Retract the intake (rollers keep running until fully stowed)
     */
    public void retract() {
        intakeEnabled = false;
        setGoal(Goal.STOWED);
        currentState = IntakeState.RETRACTING;
        // Don't stop rollers - they'll stop automatically in periodic() when stowed
    }
    
    /**
     * Toggle intake state
     */
    public void toggle() {
        if (goal == Goal.DEPLOYED) {
        retract();
        } 
        else {
        deploy();
        }
    }
    
    /**
     * Stop all intake motors
     */
    public void stop() {
        intakeEnabled = false;
        currentState = IntakeState.STOWED; 
        extensionIO.stop();
        rollerIO.stop();
    }
    
    /**
     * Run rollers in reverse to eject game piece
     */
    public void eject() {
        rollerIO.setDutyCycle(IntakeConstants.rollerEjectDutyCycle);
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

    public boolean isHomed(){
        return homed;
    }
    
      public void setGoal(Goal newGoal) {
        if (newGoal != goal) {
            goal = newGoal;
            goalChangeTimestampSec = Timer.getFPGATimestamp();
            crashDebouncer.calculate(false); // reset debounce state on new motion
            crashLatched = false;
        }

        goalPosRot = (goal == Goal.DEPLOYED)
            ? IntakeConstants.extensionDeployedPosRot.get()
            : IntakeConstants.extensionStowedPosRot.get();
    }

    public double getRollerVolts(){
        return rollerInputs.appliedVolts;
    }

    // ==================== HOMING HELPERS ====================

    /**
     * Reset homing state - call at start of homing sequence
     */
    public void resetHomingState() {
        homed = false;
        crashLatched = false;
    }

    /**
     * Set extension duty cycle directly - used for homing
     */
    public void setExtensionDutyCycle(double dutyCycle) {
        extensionIO.setDutyCycle(dutyCycle);
    }

    /**
     * Stop the extension motor
     */
    public void stopExtension() {
        extensionIO.stop();
    }


    /**
     * Zero extension encoder at stowed position - call when at hard stop
     */
    public void zeroExtensionAtStowed() {
        extensionIO.setPosition(IntakeConstants.extensionStowedPosRot.get());
        homed = true;
        setGoal(Goal.STOWED);
        Logger.recordOutput("Intake/Extension/Homed", true);
    }

    public void setPosition(){
        extensionIO.setPosition(IntakeConstants.extensionStowedPosRot.get()/2);
    }

    /**
     * Get the intake homing command
     */
    public Command homingSequence() {
        return new IntakeHomingCommand(this);
    }

    // ==================== JIGGLE HELPERS ====================

    /**
     * Set goal position directly for jiggle (bypasses state machine)
     */
    public void setGoalPosition(double positionRot) {
        goalPosRot = positionRot;
    }

    /**
     * Check if extension is at stowed position
     */
    public boolean isAtStowed() {
        return Math.abs(extensionInputs.positionRotations - IntakeConstants.extensionStowedPosRot.get()) 
            < IntakeConstants.extensionAllowedErrorRot.get();
    }

    /**
     * Get current extension position in rotations
     */
    public double getExtensionPosition() {
        return extensionInputs.positionRotations;
    }

    /**
     * Check if extension is at a specific position within tolerance
     */
    public boolean isAtPosition(double targetPosRot) {
        return Math.abs(extensionInputs.positionRotations - targetPosRot) 
            < IntakeConstants.extensionAllowedErrorRot.get();
    }

    /**
     * Check if extension is at deployed position
     */
    public boolean isAtDeployed() {
        return Math.abs(extensionInputs.positionRotations - IntakeConstants.extensionDeployedPosRot.get()) 
            < IntakeConstants.extensionAllowedErrorRot.get();
    }

    /**
     * Get the current goal
     */
    public Goal getGoal() {
        return goal;
    }

    /** For testing - bypass homing requirement */
    public void forceExtensionHomed() {
        homed = true;
        Logger.recordOutput("Intake/Extension/Homed", true);
    }

}