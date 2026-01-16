package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO.FlywheelIOInputs;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import frc.robot.subsystems.Shooter.Hood.HoodIO.HoodIOInputs;
import frc.robot.subsystems.Shooter.Kickup.KickupIO;
import frc.robot.subsystems.Shooter.Kickup.KickupIO.KickupIOInputs;
import frc.robot.subsystems.Shooter.ShooterConstants.KickupConstants;
import frc.robot.utility.ShootingCalculator;
import frc.robot.utility.ShootingCalculator.ShootingSolution;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;

public class ShooterBase extends SubsystemBase {
    
    public enum ShooterState {
        IDLE,
        SPINNING_UP,   // Flywheel getting up to speed
        READY,         // Flywheel at speed, ready to shoot
        SHOOTING,      // Actively feeding and shooting
        ADJUSTING_HOOD // Hood moving to position
    }
    
    // IO layers
    private final FlywheelIO flywheelIO;
    private final HoodIO hoodIO;
    private final KickupIO kickupIO;
    
    // Inputs (using base classes, not AutoLogged)
    private final FlywheelIOInputs flywheelInputs = new FlywheelIOInputs();
    private final HoodIOInputs hoodInputs = new HoodIOInputs();
    private final KickupIOInputs kickupInputs = new KickupIOInputs();
    
    private ShooterState currentState = ShooterState.IDLE;
    // Hood homing state
    private boolean hoodHomed = false;
    
    public ShooterBase(FlywheelIO flywheelIO, HoodIO hoodIO, KickupIO kickupIO) {
        this.flywheelIO = flywheelIO;
        this.hoodIO = hoodIO;
        this.kickupIO = kickupIO;
    }
    
    @Override
    public void periodic() {
        // Update inputs from IO layers
        flywheelIO.updateInputs(flywheelInputs);
        hoodIO.updateInputs(hoodInputs);
        kickupIO.updateInputs(kickupInputs);
        
        // Manual logging - Flywheel
        Logger.recordOutput("Shooter/Flywheel/VelocityRPM", flywheelInputs.velocityRPM);
        Logger.recordOutput("Shooter/Flywheel/VelocityRotPerSec", flywheelInputs.velocityRotPerSec);
        Logger.recordOutput("Shooter/Flywheel/AppliedVolts", flywheelInputs.appliedVolts);
        Logger.recordOutput("Shooter/Flywheel/CurrentAmps", flywheelInputs.currentAmps);
        Logger.recordOutput("Shooter/Flywheel/TempCelsius", flywheelInputs.tempCelsius);
        Logger.recordOutput("Shooter/Flywheel/TargetVelocityRPM", flywheelInputs.targetVelocityRPM);
        Logger.recordOutput("Shooter/Flywheel/AtSetpoint", flywheelInputs.atSetpoint);
        
        // Manual logging - Hood
        Logger.recordOutput("Shooter/Hood/Homed", hoodHomed);
        Logger.recordOutput("Shooter/Hood/PositionRadians", hoodInputs.positionRadians);
        Logger.recordOutput("Shooter/Hood/PositionRotations", hoodInputs.positionRotations);
        Logger.recordOutput("Shooter/Hood/VelocityRotPerSec", hoodInputs.velocityRotPerSec);
        Logger.recordOutput("Shooter/Hood/AppliedVolts", hoodInputs.appliedVolts);
        Logger.recordOutput("Shooter/Hood/CurrentAmps", hoodInputs.currentAmps);
        Logger.recordOutput("Shooter/Hood/TempCelsius", hoodInputs.tempCelsius);
        Logger.recordOutput("Shooter/Hood/TargetPositionRadians", hoodInputs.targetPositionRadians);
        Logger.recordOutput("Shooter/Hood/AtSetpoint", hoodInputs.atSetpoint);
        
        // Manual logging - Kickup
        Logger.recordOutput("Shooter/Kickup/VelocityRotPerSec", kickupInputs.velocityRotPerSec);
        Logger.recordOutput("Shooter/Kickup/AppliedVolts", kickupInputs.appliedVolts);
        Logger.recordOutput("Shooter/Kickup/CurrentAmps", kickupInputs.currentAmps);
        Logger.recordOutput("Shooter/Kickup/TempCelsius", kickupInputs.tempCelsius);
        
        // Log state
        Logger.recordOutput("Shooter/State", currentState.toString());
        Logger.recordOutput("Shooter/ReadyToShoot", isReadyToShoot());
    }
    
    // ==================== FLYWHEEL CONTROL ====================
    
    /**
     * Set flywheel to target velocity
     * @param velocityRPM target velocity in RPM
     */
    public void setFlywheelVelocity(double velocityRPM) {
        currentState = ShooterState.SPINNING_UP;
        flywheelIO.setVelocity(velocityRPM);
    }
    
    /**
     * Stop the flywheel
     */
    public void stopFlywheel() {
        flywheelIO.stop();
        if (currentState == ShooterState.SPINNING_UP || currentState == ShooterState.READY) {
            currentState = ShooterState.IDLE;
        }
    }
    
    /**
     * Check if flywheel is at target velocity
     */
    public boolean isFlywheelAtSetpoint() {
        return flywheelInputs.atSetpoint;
    }
    
    /**
     * Get current flywheel velocity in RPM
     */
    public double getFlywheelVelocityRPM() {
        return flywheelInputs.velocityRPM;
    }
    
    // ==================== HOOD CONTROL ====================
    
    /**
     * Set hood to target angle
     * @param angleRadians target angle in radians
     */
    public void setHoodAngle(double angleRadians) {
        if (!hoodHomed) {
            Logger.recordOutput("Shooter/Hood/Warning", "Hood not homed - ignoring setHoodAngle");
            return;
        }
        currentState = ShooterState.ADJUSTING_HOOD;
        hoodIO.setPosition(angleRadians);
    }
    
    /**
     * Stop the hood motor
     */
    public void stopHood() {
        hoodIO.stop();
    }
    
    /**
     * Check if hood is at target angle
     */
    public boolean isHoodAtSetpoint() {
        return hoodInputs.atSetpoint;
    }
    
    /**
     * Get current hood angle in radians
     */
    public double getHoodAngleRadians() {
        return hoodInputs.positionRadians;
    }
    
    /**
     * Zero the hood encoder - call at beginning of auto and teleop
     */
    public void zeroHood() {
        hoodIO.zeroEncoder();
    }

    public boolean isHoodHomed(){
        return hoodHomed;
    }
    
    // ==================== KICKUP CONTROL ====================
    
    /**
     * Run kickup to feed game piece into shooter
     */
    public void runKickup() {
        kickupIO.setDutyCycle(KickupConstants.feedDutyCycle.get());
    }
    
    /**
     * Run kickup in reverse
     */
    public void reverseKickup() {
        kickupIO.setDutyCycle(KickupConstants.reverseDutyCycle.get());
    }
    
    /**
     * Stop the kickup
     */
    public void stopKickup() {
        kickupIO.stop();
    }
    
    // ==================== COMBINED OPERATIONS ====================
    
    /**
     * Prepare to shoot - spin up flywheel and set hood angle
     * @param velocityRPM target flywheel velocity
     * @param hoodAngleRadians target hood angle
     */
    public void prepareToShoot(double velocityRPM, double hoodAngleRadians) {
        setFlywheelVelocity(velocityRPM);
        setHoodAngle(hoodAngleRadians);
    }
    
    /**
     * Execute shot - run kickup to feed game piece
     * Only call this when flywheel is at speed!
     */
    public void shoot() {
        currentState = ShooterState.SHOOTING;
        runKickup();
    }
    
    /**
     * Stop all shooter mechanisms
     */
    public void stopAll() {
        currentState = ShooterState.IDLE;
        flywheelIO.stop();
        hoodIO.stop();
        kickupIO.stop();
    }
    
    /**
     * Check if shooter is ready to fire (flywheel at speed and hood in position)
     */
    public boolean isReadyToShoot() {
        return flywheelInputs.atSetpoint && hoodInputs.atSetpoint;
    }
    
    // ==================== STATE GETTERS ====================
    
    public ShooterState getState() {
        return currentState;
    }
    
    // ==================== COMMANDS ====================
    
    /**
     * Command to spin up flywheel and wait until at speed
     */
    public Command spinUpCommand(double velocityRPM) {
        return runOnce(() -> setFlywheelVelocity(velocityRPM))
            .andThen(run(() -> {
                if (isFlywheelAtSetpoint()) {
                    currentState = ShooterState.READY;
                }
            }))
            .until(this::isFlywheelAtSetpoint)
            .withName("Shooter Spin Up");
    }
    
    /**
     * Command to set hood angle and wait until in position
     */
    public Command setHoodCommand(double angleRadians) {
        return runOnce(() -> setHoodAngle(angleRadians))
            .andThen(run(() -> {}))
            .until(this::isHoodAtSetpoint)
            .withName("Shooter Set Hood");
    }

    /**
 * Command to home the hood by running toward the hard stop until stalled,
 * then zeroing the encoder at the minimum position.
 */
    public Command hoodHomingSequence() {
        return Commands.sequence(
            // Disable normal control, reset state
            runOnce(() -> {
                hoodHomed = false;
                currentState = ShooterState.IDLE;
            }),
            // Run toward hard stop (negative = toward min position) until stalled
            run(() -> {
                hoodIO.setDutyCycle(ShooterConstants.hoodHomingDutyCycle.get());
            })
            .until(() -> 
                Math.abs(hoodInputs.velocityRotPerSec) < ShooterConstants.hoodHomingVelThreshold.get()
                && Math.abs(hoodInputs.currentAmps) > ShooterConstants.hoodHomingCurrentThreshold.get()
            )
            .withTimeout(3.0),  // Safety timeout
            // Zero encoder at hard stop (min position)
            runOnce(() -> {
                hoodIO.setEncoderPosition(ShooterConstants.hoodMinPosRot.get());
                hoodHomed = true;
                Logger.recordOutput("Shooter/Hood/Homed", true);
            }),
            // Stop the motor
            runOnce(() -> hoodIO.stop())
        ).withName("Hood Homing");
    }
    
    /**
     * Command to prepare shooter (flywheel + hood) and wait until ready
     */
    public Command prepareCommand(double velocityRPM, double hoodAngleRadians) {
        return runOnce(() -> prepareToShoot(velocityRPM, hoodAngleRadians))
            .andThen(run(() -> {
                if (isReadyToShoot()) {
                    currentState = ShooterState.READY;
                }
            }))
            .until(this::isReadyToShoot)
            .withName("Shooter Prepare");
    }
    
    /**
     * Command to shoot (runs kickup until interrupted)
     * Should only be used after prepareCommand completes
     */
    public Command shootCommand() {
        return runOnce(this::shoot)
            .andThen(run(() -> {}))
            .finallyDo((interrupted) -> stopKickup())
            .withName("Shooter Shoot");
    }
    
    /**
     * Command to shoot for a specific duration
     */
    public Command shootForTimeCommand(double seconds) {
        return runOnce(this::shoot)
            .andThen(run(() -> {}).withTimeout(seconds))
            .finallyDo((interrupted) -> stopKickup())
            .withName("Shooter Shoot Timed");
    }
    
    /**
     * Full shooting sequence - prepare then shoot
     */
    public Command fullShootCommand(double velocityRPM, double hoodAngleRadians, double shootDurationSeconds) {
        return prepareCommand(velocityRPM, hoodAngleRadians)
            .andThen(shootForTimeCommand(shootDurationSeconds))
            .finallyDo((interrupted) -> stopAll())
            .withName("Shooter Full Sequence");
    }
    
    /**
     * Command to stop all shooter mechanisms
     */
    public Command stopCommand() {
        return runOnce(this::stopAll).withName("Shooter Stop");
    }
    
    /**
     * Command to zero hood encoder
     */
    public Command zeroHoodCommand() {
        return runOnce(this::zeroHood).withName("Shooter Zero Hood");
    }

    /**
 * Continuously update shooter based on distance to hub.
 * Use this while driving to keep shooter ready.
 */
public Command autoAimCommand(Supplier<Pose2d> poseSupplier, BooleanSupplier isRedAllianceSupplier) {
    return run(() -> {
        ShootingSolution solution = ShootingCalculator.calculateSolution(
            poseSupplier.get(), 
            isRedAllianceSupplier.getAsBoolean()
        );
        
        setFlywheelVelocity(solution.flywheelRPM);
        setHoodAngle(solution.hoodAngleRadians);
        
        Logger.recordOutput("Shooter/AutoAim/Distance", solution.distanceMeters);
        Logger.recordOutput("Shooter/AutoAim/HoodAngleDeg", solution.getHoodAngleDegrees());
        Logger.recordOutput("Shooter/AutoAim/FlywheelRPM", solution.flywheelRPM);
    })
    .withName("Shooter Auto Aim");
}

/**
 * Auto-aim and shoot when ready.
 */
public Command autoAimAndShootCommand(
        Supplier<Pose2d> poseSupplier, 
        BooleanSupplier isRedAllianceSupplier,
        double shootDurationSeconds) {
    return run(() -> {
        ShootingSolution solution = ShootingCalculator.calculateSolution(
            poseSupplier.get(), 
            isRedAllianceSupplier.getAsBoolean()
        );
        
        setFlywheelVelocity(solution.flywheelRPM);
        setHoodAngle(solution.hoodAngleRadians);
        
        Logger.recordOutput("Shooter/AutoAim/Distance", solution.distanceMeters);
        Logger.recordOutput("Shooter/AutoAim/HoodAngleDeg", solution.getHoodAngleDegrees());
        Logger.recordOutput("Shooter/AutoAim/FlywheelRPM", solution.flywheelRPM);
    })
    .until(this::isReadyToShoot)
    .andThen(shootForTimeCommand(shootDurationSeconds))
    .finallyDo((interrupted) -> stopAll())
    .withName("Shooter Auto Aim and Shoot");
}

/**
 * Get shooting solution for current position.
 */
public ShootingSolution getShootingSolution(Pose2d robotPose, boolean isRedAlliance) {
    return ShootingCalculator.calculateSolution(robotPose, isRedAlliance);
}
}