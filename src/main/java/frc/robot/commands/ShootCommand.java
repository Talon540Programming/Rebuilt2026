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

    private boolean flywheelSpunUp = false;
    // Fuel simulation timing
    private static final double fuelSpawnInterval = 0.125; // 8 balls per second
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
                        BooleanSupplier isPassingEnabledSupplier) {
        this.shooter = shooter;
        this.index = index;
        this.poseSupplier = poseSupplier;
        this.velocitySupplier = velocitySupplier;
        this.isRedAllianceSupplier = isRedAllianceSupplier;
        this.isPassingEnabledSupplier = isPassingEnabledSupplier;
        
        // Require both subsystems
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        flywheelSpunUp = false;
        timeSinceLastSpawn = fuelSpawnInterval; // Spawn immediately on first feed
    }
    
    @Override
    public void execute() {
        double flywheelRPM;
        double hoodAngleRadians;
        
        // Check if we're in passing mode or hub shooting mode
        if (isPassingEnabledSupplier.getAsBoolean()) {
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
        if (!flywheelSpunUp) {
            if (shooter.isFlywheelAtSetpoint() && shooter.getFlywheelVelocityRPM() > ShooterConstants.flywheelVelToleranceRPM.get()) {
                // Flywheel just reached speed - mark as spun up
                flywheelSpunUp = true;
            }
            Logger.recordOutput("ShootCommand/FlywheelSpunUp", flywheelSpunUp);
            // Don't feed until spun up
            return;
        }
        
        Logger.recordOutput("ShootCommand/FlywheelSpunUp", flywheelSpunUp);
        
        // Once spun up, continuously run index and kickup
        index.feed();
        shooter.runKickup();

        // Spawn fuel in simulation
        if (Robot.isSimulation()) {
            timeSinceLastSpawn += loopPeriod;
            if (timeSinceLastSpawn >= fuelSpawnInterval) {
                timeSinceLastSpawn = 0.0;
                spawnSimulatedFuel();
            }
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop shooter - index will return to default command behavior
        shooter.stopAll();
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
     */
    private void spawnSimulatedFuel() {
        Pose2d robotPose = poseSupplier.get();
        boolean isRed = isRedAllianceSupplier.getAsBoolean();
        
        // Calculate distance to hub for velocity/angle calculation
        double distance = ShootingCalculator.getDistanceToHub(robotPose, isRed);
        
        // Get shooter position in field coordinates
        Translation2d shooterPos2d = ShootingCalculator.getShooterPosition(robotPose);
        double shooterHeight = Units.inchesToMeters(ShootingConstants.shooterHeightInches);
        Translation3d spawnPosition = new Translation3d(
            shooterPos2d.getX(),
            shooterPos2d.getY(),
            shooterHeight
        );
        
        // Calculate launch velocity
        Translation3d launchVelocity = ShootingCalculator.calculateLaunchVelocity(robotPose, distance);
        
        // Spawn the fuel
        FuelSim.getInstance().spawnFuel(spawnPosition, launchVelocity);
        
        Logger.recordOutput("ShootCommand/SimFuelSpawned", true);
        Logger.recordOutput("ShootCommand/SimSpawnPos", spawnPosition);
        Logger.recordOutput("ShootCommand/SimLaunchVel", launchVelocity);
    }
}