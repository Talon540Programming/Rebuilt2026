package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.FuelSim;
import frc.robot.utility.ShootingCalculator;
import frc.robot.utility.ShootingCalculator.ShootingSolution;
import frc.robot.subsystems.Index.IndexBase;
import frc.robot.subsystems.Shooter.ShooterBase;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.utility.ShootingCalculator.PassingSolution;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.Constants.EmergencyModeConstants;
import frc.robot.Constants.ShootingConstants;

import org.littletonrobotics.junction.Logger;

/**
 * Command that coordinates shooter, index, and kickup during shooting with auto-aim.
 * - Continuously updates flywheel RPM and hood angle based on distance to hub
 * - Spins up flywheel and waits until at speed (first time only)
 * - Once at speed, runs index and kickup to feed balls continuously
 * - Keeps everything running until interrupted
 */
public class ShootCommand extends Command {
    
    private final ShooterBase shooter;
    private final IndexBase index;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> velocitySupplier;
    private final BooleanSupplier isRedAllianceSupplier;
    private final BooleanSupplier isPassingEnabledSupplier;
    private final BooleanSupplier isEmergencyShootingSupplier;
    private final BooleanSupplier isEmergencyPassingSupplier;
    private final Runnable onShootingEndCallback;
    private final Supplier<Double> currentHeadingSupplier;
    private final Supplier<Double> targetHeadingSupplier;
    private final Supplier<Boolean> headingAtTargetSupplier;

    private boolean readyToShoot = false;
    // Fuel simulation timing
    private static final double fuelSpawnInterval = .1; // 10 balls per second
    private double timeSinceLastSpawn = 0.0;
    private static final double loopPeriod = 0.02; // 20ms loop
    
    /**
     * Creates a ShootCommand with auto-aiming based on robot position and velocity.
     * 
     * @param shooter The shooter subsystem
     * @param index The index subsystem
     * @param poseSupplier Supplier for current robot pose
     * @param velocitySupplier Supplier for field-relative velocity
     * @param isRedAllianceSupplier Supplier for alliance color
     * @param isPassingEnabledSupplier Supplier for whether passing mode is active
     */
    public ShootCommand(ShooterBase shooter, IndexBase index, 
                        Supplier<Pose2d> poseSupplier, 
                        Supplier<ChassisSpeeds> velocitySupplier,
                        BooleanSupplier isRedAllianceSupplier,
                        BooleanSupplier isPassingEnabledSupplier,
                        BooleanSupplier isEmergencyShootingSupplier,
                        BooleanSupplier isEmergencyPassingSupplier,
                        Runnable onShootingEndCallback,
                        Supplier<Double> currentHeadingSupplier,
                        Supplier<Double> targetHeadingSupplier,
                        Supplier<Boolean> headingAtTargetSupplier) {
        this.shooter = shooter;
        this.index = index;
        this.poseSupplier = poseSupplier;
        this.velocitySupplier = velocitySupplier;
        this.isRedAllianceSupplier = isRedAllianceSupplier;
        this.isPassingEnabledSupplier = isPassingEnabledSupplier;
        this.isEmergencyShootingSupplier = isEmergencyShootingSupplier;
        this.isEmergencyPassingSupplier = isEmergencyPassingSupplier;
        this.onShootingEndCallback = onShootingEndCallback;
        this.currentHeadingSupplier = currentHeadingSupplier;
        this.targetHeadingSupplier = targetHeadingSupplier;
        this.headingAtTargetSupplier = headingAtTargetSupplier;
        
        // Require both subsystems
        addRequirements(shooter, index);
    }
    
    @Override
    public void initialize() {
        readyToShoot = false;
        timeSinceLastSpawn = fuelSpawnInterval; // Spawn immediately on first feed
    }
    
    @Override
    public void execute() {
        double flywheelRPM;
        double hoodAngleRadians;
        
        // Check for emergency mode first
        if (isEmergencyShootingSupplier.getAsBoolean()) {
            // Emergency shooting mode - use preset values
            flywheelRPM = EmergencyModeConstants.shootingRPM;
            hoodAngleRadians = Math.toRadians(EmergencyModeConstants.shootingHoodAngleDegrees);
            
            Logger.recordOutput("ShootCommand/Mode", "EmergencyShooting");
        } else if (isEmergencyPassingSupplier.getAsBoolean()) {
            // Emergency passing mode - use preset values
            flywheelRPM = EmergencyModeConstants.passingRPM;
            hoodAngleRadians = Math.toRadians(EmergencyModeConstants.passingHoodAngleDegrees);
            
            Logger.recordOutput("ShootCommand/Mode", "EmergencyPassing");
        } else if (isPassingEnabledSupplier.getAsBoolean()) {
            // Passing mode - use passing calculator
            PassingSolution passingSolution = ShootingCalculator.calculatePassingSolution(
                poseSupplier.get(),
                isRedAllianceSupplier.getAsBoolean()
            );
            flywheelRPM = passingSolution.flywheelRPM;
            hoodAngleRadians = passingSolution.hoodAngleRadians;
            
            Logger.recordOutput("ShootCommand/Mode", "Passing");
            Logger.recordOutput("ShootCommand/PassingValid", passingSolution.isValid);
        } else {
            // Hub shooting mode - use movement-compensated calculator
            ShootingSolution solution = ShootingCalculator.calculateSolutionWithMovement(
                poseSupplier.get(),
                velocitySupplier.get(),
                isRedAllianceSupplier.getAsBoolean()
            );
            flywheelRPM = solution.flywheelRPM;
            hoodAngleRadians = solution.hoodAngleRadians;
            
            Logger.recordOutput("ShootCommand/Mode", "Hub");
        }
        
        // Always update flywheel and hood targets (continuous auto-aim)
        shooter.setFlywheelVelocity(flywheelRPM);
        shooter.setHoodAngle(hoodAngleRadians);
        
        // Wait for flywheel to spin up the FIRST time only
       // Wait for all conditions to be met the FIRST time only
        if (!readyToShoot) {
            boolean flywheelReady = shooter.isFlywheelAtSetpoint() && 
                shooter.getFlywheelVelocityRPM() > ShooterConstants.flywheelVelToleranceRPM.get();
            boolean hoodReady = shooter.isHoodAtSetpoint();
            boolean headingReady = headingAtTargetSupplier.get();
            
            Logger.recordOutput("ShootCommand/FlywheelReady", flywheelReady);
            Logger.recordOutput("ShootCommand/HoodReady", hoodReady);
            Logger.recordOutput("ShootCommand/HeadingReady", headingReady);
            Logger.recordOutput("ShootCommand/CurrentHeading", Math.toDegrees(currentHeadingSupplier.get()));
            Logger.recordOutput("ShootCommand/TargetHeading", Math.toDegrees(targetHeadingSupplier.get()));
            
            if (flywheelReady && hoodReady && headingReady) {
                readyToShoot = true;
            }
            Logger.recordOutput("ShootCommand/ReadyToShoot", readyToShoot);
            // Don't feed until ready
            return;
        }
        
        Logger.recordOutput("ShootCommand/ReadyToShoot", readyToShoot);
        
        // Once spun up, continuously run index and kickup
        index.feed();
        shooter.runKickup();

        /// Spawn fuel in simulation
        if (Robot.isSimulation()) {
            timeSinceLastSpawn += loopPeriod;
            if (timeSinceLastSpawn >= fuelSpawnInterval) {
                timeSinceLastSpawn = 0.0;
                spawnSimulatedFuel(shooter.getFlywheelVelocityRPM(), hoodAngleRadians);
                shooter.simulateFlywheelShot();
            }
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop shooter and index
        shooter.stopAll();
        index.stop();
        
        // Revert from hood retract mode if needed
        if (onShootingEndCallback != null) {
            onShootingEndCallback.run();
        }
    }
    
    @Override
    public boolean isFinished() {
        // Run until interrupted by driver
        return false;
    }
    
    @Override
    public String getName() {
        return "ShootCommand";
    }
/**
     * Spawns a simulated fuel ball at the shooter position with calculated velocity.
     * Uses the actual RPM and hood angle being sent to the shooter.
     * 
     * @param flywheelRPM The flywheel RPM being commanded
     * @param hoodAngleRadians The hood angle being commanded (radians)
     */
    private void spawnSimulatedFuel(double flywheelRPM, double hoodAngleRadians) {
        Pose2d robotPose = poseSupplier.get();
        
        // Get shooter position in field coordinates
        Translation2d shooterPos2d = ShootingCalculator.getShooterPosition(robotPose);
        double shooterHeight = Units.inchesToMeters(ShootingConstants.shooterHeightInches);
        Translation3d spawnPosition = new Translation3d(
            shooterPos2d.getX(),
            shooterPos2d.getY(),
            shooterHeight
        );
        
        // Convert RPM to linear velocity (m/s)
        // RPM = (omega * 60) / (2 * PI)
        // omega = RPM * 2 * PI / 60
        // v = omega * r
        double wheelRadiusMeters = Units.inchesToMeters(ShootingConstants.flywheelRadiusInches);
        double omegaRadPerSec = flywheelRPM * 2.0 * Math.PI / 60.0;
        double velocityMPS = omegaRadPerSec * wheelRadiusMeters;
        
        // Hood angle is measured from vertical (0 = straight up, 54 = relatively flat)
        // Launch angle from horizontal = 90 - hood angle
        double launchAngle = (Math.PI / 2) - hoodAngleRadians;
        
        // Calculate horizontal and vertical components
        double horizontalSpeed = velocityMPS * Math.cos(launchAngle);
        double verticalSpeed = velocityMPS * Math.sin(launchAngle);
        
        // Get robot heading (direction the robot is facing)
        double shooterAngleOffset = -frc.robot.Constants.HeadingPID.shooterThetaOffset.get();
        double heading = robotPose.getRotation().getRadians() + shooterAngleOffset;
        
        // Split horizontal speed into X and Y based on robot heading
        double vx = horizontalSpeed * Math.cos(heading);
        double vy = horizontalSpeed * Math.sin(heading);
        double vz = verticalSpeed;
        
        Translation3d launchVelocity = new Translation3d(vx, vy, vz);
        
        // Spawn the fuel
        FuelSim.getInstance().spawnFuel(spawnPosition, launchVelocity);
        
        Logger.recordOutput("ShootCommand/SimFuelSpawned", true);
        Logger.recordOutput("ShootCommand/SimSpawnPos", spawnPosition);
        Logger.recordOutput("ShootCommand/SimLaunchVel", launchVelocity);
        Logger.recordOutput("ShootCommand/SimRPM", flywheelRPM);
        Logger.recordOutput("ShootCommand/SimHoodAngleDeg", Math.toDegrees(hoodAngleRadians));
    }
}