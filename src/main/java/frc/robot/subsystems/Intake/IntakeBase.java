package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.Intake.Pivot.PivotIO;
import frc.robot.subsystems.Intake.Pivot.PivotIO.PivotIOInputs;
import frc.robot.subsystems.Intake.Pivot.PivotIOSim;
import frc.robot.subsystems.Intake.Roller.RollerIO;
import frc.robot.subsystems.Intake.Roller.RollerIO.RollerIOInputs;
import frc.robot.subsystems.Intake.Roller.RollerIOSim;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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
        new Debouncer(IntakeConstants.pivotCrashDebounceSecs.get(), DebounceType.kRising);

    private Goal goal = Goal.STOWED;
    private double goalPosRot = IntakeConstants.pivotStowedPosRot.get();

    
    private final PivotIO pivotIO;
    private final RollerIO rollerIO;
    
    // Use the base inputs class instead of AutoLogged
    private final PivotIOInputs pivotInputs = new PivotIOInputs();
    private final RollerIOInputs rollerInputs = new RollerIOInputs();
    
    private IntakeState currentState = IntakeState.STOWED;
    private boolean intakeEnabled = false;

    private boolean homed = false;

    // Track goal changes so crash detection doesn’t trip during normal motion
    private double goalChangeTimestampSec = 0.0;

    // If you want: latch a crash for driver visibility
    private boolean crashLatched = false;

    
    public IntakeBase(PivotIO pivotIO, RollerIO rollerIO) {
        this.pivotIO = pivotIO;
        this.rollerIO = rollerIO;
        crashDebouncer = new Debouncer(IntakeConstants.pivotCrashDebounceSecs.get(), DebounceType.kRising);
    }
    
    @Override
    public void periodic() {
        // Update inputs from IO layers
        pivotIO.updateInputs(pivotInputs);
        rollerIO.updateInputs(rollerInputs);

        if (edu.wpi.first.wpilibj.DriverStation.isEnabled() && homed) {
            pivotIO.runMotionMagicPosition(goalPosRot, 0.0);
            } 
            else if (!edu.wpi.first.wpilibj.DriverStation.isEnabled()) {
                pivotIO.stop();
        }

        final double errorRot = goalPosRot - pivotInputs.positionRotations;
        final boolean atPivotGoal =
            Math.abs(errorRot) < IntakeConstants.pivotAllowedErrorRot.get()
            && Math.abs(pivotInputs.velocityRotPerSec) < IntakeConstants.pivotAllowedVelRotPerSec.get();
   
        // Update state machine based on goal + atGoal
        if (goal == Goal.DEPLOYED) {
            currentState = atPivotGoal ? IntakeState.DEPLOYED : IntakeState.DEPLOYING;
        } 
        else {
            currentState = atPivotGoal ? IntakeState.STOWED : IntakeState.RETRACTING;
        }

        final double nowSec = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        final boolean settledAfterGoalChange =
            (nowSec - goalChangeTimestampSec) > IntakeConstants.pivotCrashIgnoreAfterGoalChangeSecs.get();

        final boolean crashCondition =
            homed
            && edu.wpi.first.wpilibj.DriverStation.isEnabled()
            && goal == Goal.DEPLOYED
            && settledAfterGoalChange
            && Math.abs(errorRot) > IntakeConstants.pivotCrashMinErrorRot.get()
            && Math.abs(pivotInputs.velocityRotPerSec) < IntakeConstants.pivotCrashMaxVelRotPerSec.get()
            && pivotInputs.currentAmps > IntakeConstants.pivotCrashCurrentAmps.get();

        boolean crashed = crashDebouncer.calculate(crashCondition);

       
        if (crashed && !crashLatched) {
            crashLatched = true;
            Logger.recordOutput("Intake/CrashDetected", true);

            // Auto-stow to protect mechanism
            retract();
        } else {
            Logger.recordOutput("Intake/CrashDetected", false);
        }

        Logger.recordOutput("Intake/Pivot/ErrorRot", errorRot);
        Logger.recordOutput("Intake/Pivot/AtGoal", atPivotGoal);
        Logger.recordOutput("Intake/CrashLatched", crashLatched);

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

        // Simulation controls (only in sim)
        if (Robot.isSimulation()) {
            // Read collision trigger from dashboard
            boolean triggerCollision = SmartDashboard.getBoolean("Sim/TriggerIntakeCollision", false);
            if (pivotIO instanceof PivotIOSim) {
                    ((PivotIOSim) pivotIO).simulateCollision(triggerCollision);
            }
    
            // Read game piece trigger
            boolean triggerGamePiece = SmartDashboard.getBoolean("Sim/TriggerGamePiece", false);
            if (rollerIO instanceof RollerIOSim) {
                ((RollerIOSim) rollerIO).simulateGamePieceContact(triggerGamePiece);
            }
        }
    }
    
    /**
     * Deploy the intake and run rollers
     */
    public void deploy() {
        intakeEnabled = true;
        setGoal(Goal.DEPLOYED);
        currentState = IntakeState.DEPLOYING;
        rollerIO.setDutyCycle(IntakeConstants.kRollerIntakeDutyCycle);
    }

    
    /**
     * Retract the intake and stop rollers
     */
    public void retract() {
    intakeEnabled = false;
    setGoal(Goal.STOWED);
    currentState = IntakeState.RETRACTING;
    rollerIO.stop();
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

    public boolean isHomed(){
        return homed;
    }
    
      public void setGoal(Goal newGoal) {
        if (newGoal != goal) {
            goal = newGoal;
            goalChangeTimestampSec = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            crashDebouncer.calculate(false); // reset debounce state on new motion
            crashLatched = false;
        }

        goalPosRot = (goal == Goal.DEPLOYED)
            ? IntakeConstants.pivotDeployedPosRot.get()
            : IntakeConstants.pivotStowedPosRot.get();
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

    public Command homingSequence() {
        return Commands.sequence(
            // Disable normal control, reset state
            runOnce(() -> {
                homed = false;
                crashLatched = false;
            }),
            // Run toward hard stop until stalled
            run(() -> {
                pivotIO.setDutyCycle(IntakeConstants.kPivotHomingDutyCycle);  // e.g., -0.15
            })
            .until(() -> 
                Math.abs(pivotInputs.velocityRotPerSec) < IntakeConstants.kPivotHomingVelThreshold
                && Math.abs(pivotInputs.currentAmps) > IntakeConstants.kPivotHomingCurrentThreshold
            )
            .withTimeout(2.0),  // Safety timeout
            // Zero encoder at hard stop
            runOnce(() -> {
                pivotIO.setPosition(IntakeConstants.pivotStowedPosRot.get());
                homed = true;
                setGoal(Goal.STOWED);
            })
        ).withName("Intake Homing");
    }
}