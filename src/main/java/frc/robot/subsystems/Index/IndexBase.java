package frc.robot.subsystems.Index;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Index.IndexIO.IndexIOInputs;

public class IndexBase extends SubsystemBase {
    
    public enum IndexState {
        STOPPED,
        INDEXING,      // Moving game piece toward shooter
        REVERSING  
    }
    
    private final IndexIOInputs inputs = new IndexIOInputs();
    private final IndexIO io;
    
    private IndexState currentState = IndexState.STOPPED;

    
    public IndexBase(IndexIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        // Update inputs from IO layer
        io.updateInputs(inputs);
        
        // Log state
        Logger.recordOutput("Index/State", currentState.toString());
    }
    
    // ==================== CONTROL METHODS ====================
    
    /**
     * Run index in reverse to back out a game piece
     */
    public void reverse() {
        currentState = IndexState.REVERSING;
        io.setDutyCycle(IndexConstants.reverseDutyCycle.get());
    }
    
    /**
     * Run index for feeding during shooting (same speed as indexing)
     */
    public void feed() {
        currentState = IndexState.INDEXING;
        io.setDutyCycle(IndexConstants.indexDutyCycle.get());
    }
    
    /**
     * Stop the index
     */
    public void stop() {
        currentState = IndexState.STOPPED;
        io.stop();
    }
    
    // ==================== STATE GETTERS ====================
    
    public IndexState getState() {
        return currentState;
    }
    
    public boolean isRunning() {
        return currentState != IndexState.STOPPED;
    }
    
}
