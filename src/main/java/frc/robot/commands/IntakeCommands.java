package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Wrist.Wrist;

/**
 * Command factory for intake-related operations.
 */
public class IntakeCommands {
    
    /**
     * Command 1: Runs intake rollers to collect fuel from the floor.
     * Sets intake to intake mode until commanded to stop.
     * 
     * @param intake The intake subsystem
     * @return Command to run intake
     */
    public static Command runIntakeRollers(Intake intake) {
        return Commands.run(
            () -> intake.intake(),
            intake
        ).withName("RunIntakeRollers");
    }
    
    /**
     * Command 2: Deploys the wrist to extend intake outside the robot.
     * Moves wrist to deployed position for collecting fuel.
     * 
     * @param wrist The wrist subsystem
     * @return Command to deploy wrist
     */
    public static Command deployWrist(Wrist wrist) {
        return Commands.runOnce(
            () -> wrist.deploy(),
            wrist
        ).withName("DeployWrist");
    }
    
    /**
     * Command 3: Retracts the wrist back into the robot.
     * Moves wrist to stowed position inside robot frame.
     * 
     * @param wrist The wrist subsystem
     * @return Command to retract wrist
     */
    public static Command retractWrist(Wrist wrist) {
        return Commands.runOnce(
            () -> wrist.retract(),
            wrist
        ).andThen(
            Commands.waitUntil(() -> wrist.isStowed())
        ).withName("RetractWrist");
    }
    
    /**
     * Command 5: Runs the indexer to move fuel through the hopper.
     * Continues running until commanded to stop.
     * 
     * @param indexer The indexer subsystem
     * @return Command to run indexer
     */
    public static Command runIndexer(Indexer indexer) {
        return Commands.run(
            () -> indexer.index(),
            indexer
        ).withName("RunIndexer");
    }
    
    /**
     * Complete intake sequence: Deploy wrist, run intake and indexer.
     * This is the full fuel collection operation.
     * 
     * @param intake The intake subsystem
     * @param indexer The indexer subsystem
     * @param wrist The wrist subsystem
     * @return Command to run full intake sequence
     */
    public static Command fullIntakeSequence(Intake intake, Indexer indexer, Wrist wrist) {
        return Commands.sequence(
            deployWrist(wrist),
            Commands.waitUntil(() -> wrist.isDeployed()),
            Commands.parallel(
                runIntakeRollers(intake),
                runIndexer(indexer)
            )
        ).withName("FullIntakeSequence");
    }
    
    /**
     * Stops all intake operations and retracts wrist.
     * 
     * @param intake The intake subsystem
     * @param indexer The indexer subsystem
     * @param wrist The wrist subsystem
     * @return Command to stop intake
     */
    public static Command stopIntake(Intake intake, Indexer indexer, Wrist wrist) {
        return Commands.parallel(
            Commands.runOnce(() -> intake.stop(), intake),
            Commands.runOnce(() -> indexer.stop(), indexer),
            retractWrist(wrist)
        ).withName("StopIntake");
    }
    
    /**
     * Ejects fuel out of the robot (reverse intake).
     * 
     * @param intake The intake subsystem
     * @param indexer The indexer subsystem
     * @return Command to eject fuel
     */
    public static Command ejectFuel(Intake intake, Indexer indexer) {
        return Commands.parallel(
            Commands.run(() -> intake.eject(), intake),
            Commands.run(() -> indexer.reverse(), indexer)
        ).withName("EjectFuel");
    }
}