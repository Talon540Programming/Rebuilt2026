package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Kickup.Kickup;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionBase;
import frc.robot.Util.TrajectoryCalculator;

/**
 * Command factory for shooter-related operations.
 */
public class ShooterCommands {
    
    // Shooter wheel diameter for velocity calculations (inches)
    private static final double SHOOTER_WHEEL_DIAMETER = 4.0; // TODO: Measure actual diameter
    
    /**
     * Command 9: Sets the hood to a specific angle.
     * 
     * @param hood The hood subsystem
     * @param angleRadians Target angle in radians
     * @return Command to set hood angle
     */
    public static Command setHoodAngle(Hood hood, double angleRadians) {
        return Commands.runOnce(
            () -> hood.setAngleRadians(angleRadians),
            hood
        ).withName("SetHoodAngle");
    }
    
    /**
     * Command 10: Spins up shooter flywheels to target velocity.
     * Runs until optimal velocity is reached.
     * 
     * @param shooter The shooter subsystem
     * @param targetVelocityRPS Target bottom flywheel velocity in RPS
     * @return Command to spin up shooter
     */
    public static Command spinUpShooter(Shooter shooter, double targetVelocityRPS) {
        return Commands.runOnce(
            () -> shooter.setVelocity(targetVelocityRPS),
            shooter
        ).andThen(
            Commands.waitUntil(() -> shooter.isAtTargetVelocity())
        ).withName("SpinUpShooter");
    }
    
    /**
     * Command 7: Starts the kickup motor.
     * Changes kickup state to RUNNING.
     * 
     * @param kickup The kickup subsystem
     * @return Command to start kickup
     */
    public static Command startKickup(Kickup kickup) {
        return Commands.run(
            () -> kickup.feed(),
            kickup
        ).withName("StartKickup");
    }
    
    /**
     * Command 8: Starts kickup only after shooter reaches optimal velocity.
     * Waits for shooter to be ready before feeding fuel.
     * 
     * @param kickup The kickup subsystem
     * @param shooter The shooter subsystem
     * @return Command to start kickup when ready
     */
    public static Command startKickupWhenReady(Kickup kickup, Shooter shooter) {
        return Commands.sequence(
            Commands.waitUntil(() -> shooter.isAtTargetVelocity()),
            Commands.run(() -> kickup.feed(), kickup)
        ).withName("StartKickupWhenReady");
    }
    
    /**
     * Calculates and sets hood angle and shooter velocity based on distance to target.
     * Uses vision to determine robot position and trajectory calculations.
     * 
     * @param shooter The shooter subsystem
     * @param hood The hood subsystem
     * @param drivetrain The drivetrain for odometry
     * @param vision The vision subsystem for alliance color
     * @return Command to aim shooter
     */
    public static Command aimShooter(
            Shooter shooter, 
            Hood hood, 
            CommandSwerveDrivetrain drivetrain,
            VisionBase vision) {
        
        TrajectoryCalculator calculator = new TrajectoryCalculator();
        
        return Commands.run(() -> {
            // Get current robot pose and target pose
            var robotPose = drivetrain.getPose();
            var targetPose = TrajectoryCalculator.getTargetPose(vision.isRedAlliance());
            
            // Calculate optimal trajectory
            if (calculator.calculateTrajectory(robotPose, targetPose)) {
                // Set hood angle
                double hoodAngle = calculator.getLaunchAngleRadians();
                hood.setAngleRadians(hoodAngle);
                
                // Set shooter velocity
                double launchVelocity = calculator.getLaunchVelocityFeetPerSecond();
                double flywheelRPS = TrajectoryCalculator.velocityToFlywheelRPS(
                    launchVelocity, 
                    SHOOTER_WHEEL_DIAMETER
                );
                shooter.setVelocity(flywheelRPS);
            }
        }, shooter, hood).withName("AimShooter");
    }
    
    /**
     * Complete shooting sequence:
     * 1. Aim shooter (hood + flywheel)
     * 2. Wait for shooter to reach velocity
     * 3. Start indexer feeding
     * 4. Start kickup feeding
     * 
     * @param shooter The shooter subsystem
     * @param hood The hood subsystem
     * @param kickup The kickup subsystem
     * @param indexer The indexer subsystem
     * @param drivetrain The drivetrain
     * @param vision The vision subsystem
     * @return Command for full shooting sequence
     */
    public static Command shootSequence(
            Shooter shooter,
            Hood hood,
            Kickup kickup,
            Indexer indexer,
            CommandSwerveDrivetrain drivetrain,
            VisionBase vision) {
        
        return Commands.sequence(
            // Aim shooter
            aimShooter(shooter, hood, drivetrain, vision).withTimeout(0.5),
            
            // Wait for shooter to be ready
            Commands.waitUntil(() -> shooter.isAtTargetVelocity()),
            
            // Start feeding fuel
            Commands.parallel(
                Commands.run(() -> indexer.feed(), indexer),
                Commands.run(() -> kickup.feed(), kickup)
            )
        ).withName("ShootSequence");
    }
    
    /**
     * Stops all shooting operations.
     * 
     * @param shooter The shooter subsystem
     * @param kickup The kickup subsystem
     * @param indexer The indexer subsystem
     * @return Command to stop shooting
     */
    public static Command stopShooting(Shooter shooter, Kickup kickup, Indexer indexer) {
        return Commands.parallel(
            Commands.runOnce(() -> shooter.stop(), shooter),
            Commands.runOnce(() -> kickup.stop(), kickup),
            Commands.runOnce(() -> indexer.stop(), indexer)
        ).withName("StopShooting");
    }
    
    /**
     * Preset shot from a fixed position (no vision calculation).
     * Uses predetermined angle and velocity values.
     * 
     * @param shooter The shooter subsystem
     * @param hood The hood subsystem
     * @param kickup The kickup subsystem
     * @param indexer The indexer subsystem
     * @param hoodAngleRad Hood angle in radians
     * @param shooterVelocityRPS Shooter velocity in RPS
     * @return Command for preset shot
     */
    public static Command presetShot(
            Shooter shooter,
            Hood hood,
            Kickup kickup,
            Indexer indexer,
            double hoodAngleRad,
            double shooterVelocityRPS) {
        
        return Commands.sequence(
            // Set hood and shooter
            Commands.parallel(
                Commands.runOnce(() -> hood.setAngleRadians(hoodAngleRad), hood),
                Commands.runOnce(() -> shooter.setVelocity(shooterVelocityRPS), shooter)
            ),
            
            // Wait for ready
            Commands.waitUntil(() -> 
                shooter.isAtTargetVelocity() && hood.isAtTargetPosition()
            ),
            
            // Feed
            Commands.parallel(
                Commands.run(() -> indexer.feed(), indexer),
                Commands.run(() -> kickup.feed(), kickup)
            )
        ).withName("PresetShot");
    }
}