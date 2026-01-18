package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utility.ShootingCalculator;
import frc.robot.utility.ShootingCalculator.ShootingSolution;
import frc.robot.subsystems.Index.IndexBase;
import frc.robot.subsystems.Shooter.ShooterBase;
import frc.robot.subsystems.Shooter.ShooterConstants;

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
    private final BooleanSupplier isRedAllianceSupplier;
    
    private boolean flywheelSpunUp = false;
    
    /**
     * Creates a ShootCommand with auto-aiming based on robot position.
     * 
     * @param shooter The shooter subsystem
     * @param index The index subsystem
     * @param poseSupplier Supplier for current robot pose
     * @param isRedAllianceSupplier Supplier for alliance color
     */
    public ShootCommand(ShooterBase shooter, IndexBase index, 
                        Supplier<Pose2d> poseSupplier, BooleanSupplier isRedAllianceSupplier) {
        this.shooter = shooter;
        this.index = index;
        this.poseSupplier = poseSupplier;
        this.isRedAllianceSupplier = isRedAllianceSupplier;
        
        // Require both subsystems
        addRequirements(shooter);
    }
    
    @Override
    public void initialize() {
        flywheelSpunUp = false;
    }
    
    @Override
    public void execute() {
        // Continuously update shooter parameters based on current position
        ShootingSolution solution = ShootingCalculator.calculateSolution(
            poseSupplier.get(), 
            isRedAllianceSupplier.getAsBoolean()
        );
        
        // Always update flywheel and hood targets (continuous auto-aim)
        shooter.setFlywheelVelocity(solution.flywheelRPM);
        shooter.setHoodAngle(solution.hoodAngleRadians);
        
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
        // (Don't check isReadyToShoot again - just keep feeding)
        // Once spun up, continuously run index and kickup
        index.feed();
        shooter.runKickup();
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
}