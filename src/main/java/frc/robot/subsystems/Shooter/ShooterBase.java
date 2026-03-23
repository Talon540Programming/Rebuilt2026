package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIO.FlywheelIOInputs;
import frc.robot.subsystems.Shooter.Hood.HoodIO;
import frc.robot.subsystems.Shooter.Hood.HoodIO.HoodIOInputs;
import frc.robot.subsystems.Shooter.Kickup.KickupIO;
import frc.robot.subsystems.Shooter.Kickup.KickupIO.KickupIOInputs;
import frc.robot.Constants.ShootingConstants;
import frc.robot.Robot;
import frc.robot.commands.HoodHomingCommand;
import frc.robot.subsystems.Shooter.ShooterConstants.KickupConstants;


public class ShooterBase extends SubsystemBase {
    
    public enum ShooterState {
        IDLE,
        SPINNING_UP,   // Flywheel getting up to speed
        READY,         // Flywheel at speed, ready to shoot
        SHOOTING,      // Actively feeding and shooting
        ADJUSTING_HOOD, // Hood moving to position
        HOMING
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
    // Hood out-of-bounds detection
    private Debouncer hoodOutOfBoundsDebouncer = 
        new Debouncer(ShooterConstants.hoodOutOfBoundsDebounceSeconds.get(), DebounceType.kRising);


    
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
        Logger.recordOutput("Shooter/Flywheel/State", flywheelInputs.state.toString());        Logger.recordOutput("Shooter/Flywheel/TorqueCurrentAmps", flywheelInputs.torqueCurrentAmps);
        Logger.recordOutput("Shooter/Flywheel/ShotCount", flywheelInputs.shotCount);
       
        // Manual logging - Hood
        Logger.recordOutput("Shooter/Hood/Homed", hoodHomed);
        Logger.recordOutput("Shooter/Hood/PositionRadians", hoodInputs.positionRadians);
        Logger.recordOutput("Shooter/Hood/PositionRotations", hoodInputs.positionRotations);
        Logger.recordOutput("Shooter/Hood/PositionDegrees", Math.toDegrees(hoodInputs.positionRadians));
        Logger.recordOutput("Shooter/Hood/VelocityRotPerSec", hoodInputs.velocityRotPerSec);
        Logger.recordOutput("Shooter/Hood/AppliedVolts", hoodInputs.appliedVolts);
        Logger.recordOutput("Shooter/Hood/CurrentAmps", hoodInputs.currentAmps);
        Logger.recordOutput("Shooter/Hood/TempCelsius", hoodInputs.tempCelsius);
        Logger.recordOutput("Shooter/Hood/TargetPositionRadians", hoodInputs.targetPositionRadians);
        Logger.recordOutput("Shooter/Hood/TargetPositionDegrees", Math.toDegrees(hoodInputs.targetPositionRadians));
        Logger.recordOutput("Shooter/Hood/AtSetpoint", hoodInputs.atSetpoint);
        
        // Manual logging - Kickup
        Logger.recordOutput("Shooter/Kickup/VelocityRotPerSec", kickupInputs.velocityRotPerSec);
        Logger.recordOutput("Shooter/Kickup/AppliedVolts", kickupInputs.appliedVolts);
        Logger.recordOutput("Shooter/Kickup/CurrentAmps", kickupInputs.currentAmps);
        Logger.recordOutput("Shooter/Kickup/TempCelsius", kickupInputs.tempCelsius);
        
        // Log state
        Logger.recordOutput("Shooter/State", currentState.toString());
        Logger.recordOutput("Shooter/ReadyToShoot", isReadyToShoot());

        // Hood out-of-bounds check - trigger rehoming if hood exceeds max position
        if (DriverStation.isEnabled() && hoodHomed && currentState != ShooterState.HOMING) {
            boolean outOfBounds = hoodInputs.positionRotations > ShooterConstants.hoodMaxPositionRot.get();
            boolean outOfBoundsConfirmed = hoodOutOfBoundsDebouncer.calculate(outOfBounds);
            
            if (outOfBoundsConfirmed) {
                hoodHomed = false;
                currentState = ShooterState.HOMING;
                Logger.recordOutput("Shooter/Hood/OutOfBoundsTriggered", true);
                CommandScheduler.getInstance().schedule(hoodHomingSequence());
            }
            
            Logger.recordOutput("Shooter/Hood/OutOfBounds", outOfBounds);
        }
    }
        
    // ==================== FLYWHEEL CONTROL ====================
    
/**
     * Set flywheel to target velocity using MA-style bang-bang control.
     * Mode switching (duty cycle vs torque current) is handled in the IO layer.
     * @param velocityRPM target velocity in RPM
     * @param distanceMeters distance to hub in meters (used to select scalar)
     * @param applyScalar whether to apply the velocity scalar (false for emergency mode)
     */
    public void setFlywheelVelocity(double velocityRPM, double distanceMeters, boolean applyScalar) {
        currentState = ShooterState.SPINNING_UP;
        
        double scalar = 1.0;
        
        // Only apply scalar on real robot, not in simulation
        if (applyScalar && !Robot.isSimulation()) {
            // Check if testing scalar is set (non-zero)
            double testScalar = ShooterConstants.flywheelVelocityScalarTest.get();
            if (testScalar > 0.0) {
                // Use testing scalar for tuning
                scalar = testScalar;
            } else {
                // Use exponential function: scalar = 1.72221 * 1.07112^distance
                // scalar = 1.72221 * Math.pow(1.07112, distanceMeters);
                scalar = (0.247235 * distanceMeters) + 1.42105;
            }
        }

        if(scalar > 2.3){
            scalar = 2.3;
        }
        
        double finalVelocity = velocityRPM * scalar;
        
        Logger.recordOutput("Shooter/Flywheel/RequestedRPM", velocityRPM);
        Logger.recordOutput("Shooter/Flywheel/DistanceMeters", distanceMeters);
        Logger.recordOutput("Shooter/Flywheel/ScalarValue", scalar);
        Logger.recordOutput("Shooter/Flywheel/ScaledRPM", finalVelocity);
        
        flywheelIO.runBangBang(finalVelocity);
    }
    
    /**
     * Set flywheel to target velocity with distance-based scalar applied (normal hub shooting mode).
     * @param velocityRPM target velocity in RPM
     * @param distanceMeters distance to hub in meters
     */
    public void setFlywheelVelocity(double velocityRPM, double distanceMeters) {
        setFlywheelVelocity(velocityRPM, distanceMeters, true);
    }
    
    /**
     * Set flywheel to target velocity without distance-based scaling.
     * Used for emergency mode and passing where scalar is not applied.
     * @param velocityRPM target velocity in RPM
     * @param applyScalar whether to apply scaling (always false when using this overload)
     */
    public void setFlywheelVelocityNoDistance(double velocityRPM, boolean applyScalar) {
        currentState = ShooterState.SPINNING_UP;
        Logger.recordOutput("Shooter/Flywheel/RequestedRPM", velocityRPM);
        Logger.recordOutput("Shooter/Flywheel/ScalarZone", "None");
        Logger.recordOutput("Shooter/Flywheel/ScalarValue", 1.0);
        Logger.recordOutput("Shooter/Flywheel/ScaledRPM", velocityRPM);
        flywheelIO.runBangBang(velocityRPM);
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
     * Retract hood to minimum angle (stowed position)
     */
    public void retractHood() {
        setHoodAngle(ShootingConstants.hoodMinAngle);
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

    /**
     * Reset homing state - call at start of homing sequence
     */
    public void resetHomingState() {
        hoodHomed = false;
        currentState = ShooterState.IDLE;
    }

    /**
     * Set hood duty cycle directly - used for homing
     */
    public void setHoodDutyCycle(double dutyCycle) {
        hoodIO.setDutyCycle(dutyCycle);
    }

    /**
     * Zero hood encoder at minimum position - call when at hard stop
     */
    public void zeroHoodAtMin() {
        hoodIO.setEncoderPosition(ShooterConstants.hoodMinPositionRot.get());
        hoodHomed = true;
        Logger.recordOutput("Shooter/Hood/Homed", true);
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

    /** For simulation - bypass homing requirement */
    public void forceHoodHomed() {
        hoodHomed = true;
        Logger.recordOutput("Shooter/Hood/Homed", true);
    }
    
    // ==================== COMBINED OPERATIONS ====================
    
   /**
     * Prepare to shoot - spin up flywheel and set hood angle
     * @param velocityRPM target flywheel velocity
     * @param hoodAngleRadians target hood angle
     * @param distanceMeters distance to hub for scalar selection
     */
    public void prepareToShoot(double velocityRPM, double hoodAngleRadians, double distanceMeters) {
        setFlywheelVelocity(velocityRPM, distanceMeters);
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
   
    /**
    * Command to home the hood by running toward the hard stop until stalled,
    * then zeroing the encoder at the minimum position.
    */
    /**
 * Get the hood homing command
 */
    public Command hoodHomingSequence() {
        return new HoodHomingCommand(this).withTimeout(ShooterConstants.hoodHomingTimeoutSeconds.get() + 0.5);
    }
    
     /**
     * Simulate a shot stealing energy from the flywheel (sim only)
     */
    public void simulateFlywheelShot() {
        flywheelIO.simulateShot();
    }

    /**
     * Exit homing state - called when homing completes successfully
     */
    public void exitHomingState() {
        if (currentState == ShooterState.HOMING) {
            currentState = ShooterState.IDLE;
        }
        hoodOutOfBoundsDebouncer.calculate(false);  // Reset debouncer
        Logger.recordOutput("Shooter/Hood/OutOfBoundsTriggered", false);
    }
    
}