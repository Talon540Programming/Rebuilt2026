package frc.robot.subsystems.Index;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
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
    private boolean wantIntakeIndex = false;

    
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
     * Run index forward to move game piece toward shooter
     */
    public void index() {
        currentState = IndexState.INDEXING;
        io.setDutyCycle(IndexConstants.indexDutyCycle.get());
    }
    
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

     public void requestIntakeIndex(boolean enable){
        wantIntakeIndex = enable; 
    }

    public boolean getwantIntakeIndex(){
        return wantIntakeIndex;
    }
    
    // ==================== SENSOR METHODS ====================
    
    /**
     * Check if a game piece is detected by the CANRange sensor
     * @return true if game piece is present
     */
    public boolean hasGamePiece() {
        return inputs.hasGamePiece;
    }
    
    /**
     * Get the raw distance reading from CANRange
     * @return distance in meters
     */
    public double getDistance() {
        return inputs.distanceMeters;
    }
    
    /**
     * Check if index is ready to feed (game piece is in position)
     * @return true if ready to shoot
     */
    public boolean isReadyToFeed() {
        return inputs.hasGamePiece;
    }
    
    // ==================== STATE GETTERS ====================
    
    public IndexState getState() {
        return currentState;
    }
    
    public boolean isRunning() {
        return currentState != IndexState.STOPPED;
    }
    
    // ==================== COMMANDS ====================
    
    /**
     * Command to run index until game piece is detected
     */
    public Command indexUntilGamePieceCommand() {
        return run(this::index)
            .until(this::hasGamePiece)
            .finallyDo((interrupted) -> stop())
            .withName("Index Until Game Piece");
    }
    
    /**
     * Command to run index (manual control, runs until interrupted)
     */
    public Command indexCommand() {
        return runOnce(this::index)
            .andThen(run(() -> {}))
            .finallyDo((interrupted) -> stop())
            .withName("Index");
    }
    
    /**
     * Command to reverse index (runs until interrupted)
     */
    public Command reverseCommand() {
        return runOnce(this::reverse)
            .andThen(run(() -> {}))
            .finallyDo((interrupted) -> stop())
            .withName("Index Reverse");
    }
    
   /**
     * Command to feed during shooting (runs until interrupted)
     */
    public Command feedCommand() {
        return indexCommand().withName("Index Feed");
    }
}
