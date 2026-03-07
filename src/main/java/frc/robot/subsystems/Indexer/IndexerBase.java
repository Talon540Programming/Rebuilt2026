package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer.IndexerIO.IndexerIOInputs;

public class IndexerBase extends SubsystemBase{
    
    public enum IndexState {
        STOPPED,
        INDEXING
    }

    private final IndexerIOInputs inputs = new IndexerIOInputs();
    private final IndexerIO io;

    private IndexState currentState = IndexState.STOPPED;

    public IndexerBase(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        // Updates the inputs 
        io.updateInputs(inputs);

        // Logs the state
        Logger.recordOutput("Index/State", currentState.toString());
    }

    //---- Control Commands ----

    public void feed() {
        currentState = IndexState.INDEXING;
        io.setDutyCycle(0); // Need to update
    }

    public void stop() {
        currentState = IndexState.STOPPED;
        io.setDutyCycle(0);
    }

    //---- State Getters ----

    public IndexState getState() {
        return currentState;
    }
}
