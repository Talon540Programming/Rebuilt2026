package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.FuelSim;
import frc.robot.utility.ShootingCalculator;
import frc.robot.subsystems.Index.IndexBase;
import frc.robot.subsystems.Shooter.ShooterBase;
import frc.robot.subsystems.Shooter.ShooterConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.Constants.ShootingConstants;

import org.littletonrobotics.junction.Logger;

/**
 * Command that coordinates shooter, index, and kickup during shooting.
 * - Spins up flywheel and sets hood angle based on distance to hub
 * - Waits until flywheel at speed, hood at angle, and heading aligned
 * - Once ready, runs index and kickup to feed balls continuously
 * - Keeps everything running until interrupted
 * 
 * This command handles flywheel/hood control directly (not relying on default command)
 * so it works in both teleop and autonomous modes.
 */
public class ShootCommand extends Command {
    
    private final ShooterBase shooter;
    private final IndexBase index;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> velocitySupplier;
    private final Supplier<Double> currentHeadingSupplier;
    private final Supplier<Double> targetHeadingSupplier;
    private final Supplier<Boolean> headingAtTargetSupplier;
    private final Supplier<Boolean> emergencyModeSupplier;
    private final Supplier<Boolean> isRedAllianceSupplier;

    private boolean readyToShoot = false;
    
    // Fuel simulation timing
    private static final double fuelSpawnInterval = 0.25; // 10 balls per second
    private double timeSinceLastSpawn = 0.0;
    private static final double loopPeriod = 0.02; // 20ms loop
    
    /**
     * Creates a ShootCommand that handles flywheel spin-up, hood positioning, and feeding.
     * 
     * @param shooter The shooter subsystem
     * @param index The index subsystem
     * @param poseSupplier Supplier for current robot pose
     * @param velocitySupplier Supplier for current robot velocity (for shoot-while-moving)
     * @param currentHeadingSupplier Supplier for current robot heading
     * @param targetHeadingSupplier Supplier for target heading
     * @param headingAtTargetSupplier Supplier for whether heading is at target
     * @param emergencyModeSupplier Supplier for whether emergency mode is active
     * @param isRedAllianceSupplier Supplier for whether robot is on red alliance
     */
    public ShootCommand(ShooterBase shooter, IndexBase index, 
                        Supplier<Pose2d> poseSupplier,
                        Supplier<ChassisSpeeds> velocitySupplier,
                        Supplier<Double> currentHeadingSupplier,
                        Supplier<Double> targetHeadingSupplier,
                        Supplier<Boolean> headingAtTargetSupplier,
                        Supplier<Boolean> emergencyModeSupplier,
                        Supplier<Boolean> isRedAllianceSupplier) {
        this.shooter = shooter;
        this.index = index;
        this.poseSupplier = poseSupplier;
        this.velocitySupplier = velocitySupplier;
        this.currentHeadingSupplier = currentHeadingSupplier;
        this.targetHeadingSupplier = targetHeadingSupplier;
        this.headingAtTargetSupplier = headingAtTargetSupplier;
        this.emergencyModeSupplier = emergencyModeSupplier;
        this.isRedAllianceSupplier = isRedAllianceSupplier;
        
        // Require both shooter and index since we control flywheel/hood directly
        addRequirements(shooter, index);
    }
    
    @Override
    public void initialize() {
        readyToShoot = false;
        timeSinceLastSpawn = fuelSpawnInterval; // Spawn immediately on first feed
    }
    
    @Override
    public void execute() {
        Pose2d robotPose = poseSupplier.get();
        ChassisSpeeds velocity = velocitySupplier.get();
        boolean isRed = isRedAllianceSupplier.get();
        
        // Calculate shooting solution with movement compensation
        var solution = ShootingCalculator.calculateSolutionWithMovement(
            robotPose,
            velocity,
            isRed
        );
        
        // Command flywheel and hood
        shooter.setFlywheelVelocity(solution.flywheelRPM, solution.distanceMeters);
        shooter.setHoodAngle(solution.hoodAngleRadians);
        
        // Log shooting parameters
        Logger.recordOutput("ShootCommand/DistanceMeters", solution.distanceMeters);
        Logger.recordOutput("ShootCommand/FlywheelRPM", solution.flywheelRPM);
        Logger.recordOutput("ShootCommand/HoodAngleDeg", Math.toDegrees(solution.hoodAngleRadians));
        
        // Wait for all conditions to be met the FIRST time only
        if (!readyToShoot) {
            boolean isEmergencyMode = emergencyModeSupplier.get();
            
            // Check readiness conditions
            boolean flywheelReady = shooter.isFlywheelAtSetpoint() && 
                shooter.getFlywheelVelocityRPM() > ShooterConstants.flywheelVelToleranceRPM.get();
            boolean hoodReady = isEmergencyMode || shooter.isHoodAtSetpoint();
            boolean headingReady = isEmergencyMode || headingAtTargetSupplier.get();
            
            Logger.recordOutput("ShootCommand/EmergencyMode", isEmergencyMode);
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
        
        // Once ready, continuously run index and kickup
        index.feed();
        shooter.shoot();  // Runs kickup AND sets state to SHOOTING

        // Spawn fuel in simulation
        if (Robot.isSimulation()) {
            timeSinceLastSpawn += loopPeriod;
            if (timeSinceLastSpawn >= fuelSpawnInterval) {
                timeSinceLastSpawn = 0.0;
                spawnSimulatedFuel(shooter.getFlywheelVelocityRPM(), shooter.getHoodAngleRadians());
                shooter.simulateFlywheelShot();
            }
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop index and kickup
        index.stop();
        shooter.stopKickup();
        // Note: flywheel keeps spinning (controlled by default command in teleop)
    }
    
    @Override
    public boolean isFinished() {
        // Run until interrupted
        return false;
    }
    
    @Override
    public String getName() {
        return "ShootCommand";
    }
    
    /**
     * Spawns a simulated fuel ball at the shooter position with calculated velocity.
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
        double wheelRadiusMeters = Units.inchesToMeters(ShootingConstants.flywheelRadiusInches);
        double omegaRadPerSec = flywheelRPM * 2.0 * Math.PI / 60.0;
        double velocityMPS = omegaRadPerSec * wheelRadiusMeters;
        
        // Hood angle is measured from vertical (0 = straight up, 55 = relatively flat)
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