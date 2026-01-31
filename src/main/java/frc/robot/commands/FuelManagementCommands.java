package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerSensor;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Wrist.Wrist;

/**
 * Command factory for fuel management and counting operations.
 */
public class FuelManagementCommands {
    
    /** Maximum fuel capacity in the hopper */
    private static final int MAX_FUEL_CAPACITY = 3; // TODO: Verify actual capacity
    
    /**
     * Command 6: Monitors and counts fuel pieces as they are indexed.
     * Automatically stops intake when hopper is full.
     * 
     * @param intake The intake subsystem
     * @param indexer The indexer subsystem
     * @param wrist The wrist subsystem
     * @param sensor The indexer sensor for counting
     * @return Command to intake with fuel counting
     */
    public static Command intakeWithCounting(
            Intake intake, 
            Indexer indexer, 
            Wrist wrist,
            IndexerSensor sensor) {
        
        return Commands.sequence(
            // Deploy wrist
            Commands.runOnce(() -> wrist.deploy(), wrist),
            Commands.waitUntil(() -> wrist.isDeployed()),
            
            // Run intake and indexer until full
            Commands.parallel(
                Commands.run(() -> intake.intake(), intake),
                Commands.run(() -> indexer.index(), indexer),
                Commands.run(() -> sensor.update()) // Update sensor continuously
            ).until(() -> sensor.isFull(MAX_FUEL_CAPACITY)),
            
            // Stop and retract when full
            Commands.runOnce(() -> {
                intake.stop();
                indexer.stop();
                wrist.retract();
            }, intake, indexer, wrist)
        ).withName("IntakeWithCounting");
    }
    
    /**
     * Intakes a single piece of fuel.
     * Stops automatically when one fuel is detected.
     * 
     * @param intake The intake subsystem
     * @param indexer The indexer subsystem
     * @param wrist The wrist subsystem
     * @param sensor The indexer sensor
     * @return Command to intake single fuel
     */
    public static Command intakeSingleFuel(
            Intake intake,
            Indexer indexer,
            Wrist wrist,
            IndexerSensor sensor) {
        
        int startCount = sensor.getFuelCount();
        
        return Commands.sequence(
            // Deploy wrist
            Commands.runOnce(() -> wrist.deploy(), wrist),
            Commands.waitUntil(() -> wrist.isDeployed()),
            
            // Run until one more fuel detected
            Commands.parallel(
                Commands.run(() -> intake.intake(), intake),
                Commands.run(() -> indexer.index(), indexer),
                Commands.run(() -> sensor.update())
            ).until(() -> sensor.getFuelCount() > startCount),
            
            // Stop and retract
            Commands.runOnce(() -> {
                intake.stop();
                indexer.stop();
                wrist.retract();
            }, intake, indexer, wrist)
        ).withName("IntakeSingleFuel");
    }
    
    /**
     * Resets the fuel count.
     * Use this at match start or after scoring all fuel.
     * 
     * @param sensor The indexer sensor
     * @return Command to reset count
     */
    public static Command resetFuelCount(IndexerSensor sensor) {
        return Commands.runOnce(() -> sensor.resetCount()).withName("ResetFuelCount");
    }
    
    /**
     * Checks if hopper has fuel.
     * 
     * @param sensor The indexer sensor
     * @return true if fuel is available
     */
    public static boolean hasFuel(IndexerSensor sensor) {
        return !sensor.isEmpty();
    }
    
    /**
     * Checks if hopper is full.
     * 
     * @param sensor The indexer sensor
     * @return true if at max capacity
     */
    public static boolean hopperFull(IndexerSensor sensor) {
        return sensor.isFull(MAX_FUEL_CAPACITY);
    }
    
    /**
     * Auto-intake command that runs continuously until hopper is full.
     * Includes smart behavior:
     * - Deploys wrist when starting
     * - Retracts wrist when full
     * - Stops all mechanisms when full
     * 
     * @param intake The intake subsystem
     * @param indexer The indexer subsystem
     * @param wrist The wrist subsystem
     * @param sensor The indexer sensor
     * @return Command for automatic intaking
     */
    public static Command autoIntake(
            Intake intake,
            Indexer indexer,
            Wrist wrist,
            IndexerSensor sensor) {
        
        return Commands.sequence(
            // Only proceed if not already full
            Commands.either(
                Commands.none(),
                
                Commands.sequence(
                    // Deploy
                    Commands.runOnce(() -> wrist.deploy(), wrist),
                    
                    // Run until full
                    Commands.parallel(
                        Commands.run(() -> {
                            intake.intake();
                            indexer.index();
                            sensor.update();
                        })
                    ).until(() -> sensor.isFull(MAX_FUEL_CAPACITY)),
                    
                    // Cleanup
                    Commands.runOnce(() -> {
                        intake.stop();
                        indexer.stop();
                        wrist.retract();
                    }, intake, indexer, wrist)
                ),
                
                () -> sensor.isFull(MAX_FUEL_CAPACITY)
            )
        ).withName("AutoIntake");
    }
    
    /**
     * Manual intake control that respects fuel capacity.
     * Prevents overfilling the hopper.
     * 
     * @param intake The intake subsystem
     * @param indexer The indexer subsystem
     * @param sensor The indexer sensor
     * @return Command for manual intake with protection
     */
    public static Command manualIntake(
            Intake intake,
            Indexer indexer,
            IndexerSensor sensor) {
        
        return Commands.run(() -> {
            sensor.update();
            
            if (!sensor.isFull(MAX_FUEL_CAPACITY)) {
                intake.intake();
                indexer.index();
            } else {
                intake.stop();
                indexer.stop();
            }
        }, intake, indexer).withName("ManualIntake");
    }
}