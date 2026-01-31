package frc.robot.subsystems.Indexer;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import org.littletonrobotics.junction.Logger;

/**
 * Sensor for detecting fuel in the indexer using a TimeOfFlight (CanRange) sensor.
 * This sensor counts balls as they pass through the indexer.
 */
public class IndexerSensor {
    
    private final TimeOfFlight sensor;
    private int fuelCount = 0;
    private boolean lastSensorState = false;
    
    /** Distance threshold in mm - if sensor reads less than this, fuel is detected */
    private static final double DETECTION_THRESHOLD_MM = 100.0;
    
    /** CAN ID for the TimeOfFlight sensor */
    private static final int SENSOR_CAN_ID = 40; // TODO: Set actual CAN ID
    
    public IndexerSensor() {
        sensor = new TimeOfFlight(SENSOR_CAN_ID);
        sensor.setRangingMode(RangingMode.Short, 24); // Short range, 24ms sample time
    }
    
    /**
     * Updates the sensor and increments fuel count when fuel passes.
     * Call this periodically (in subsystem periodic()).
     */
    public void update() {
        boolean currentState = hasFuel();
        
        // Detect rising edge - fuel entering sensor range
        if (currentState && !lastSensorState) {
            fuelCount++;
            Logger.recordOutput("Indexer/FuelCount", fuelCount);
            Logger.recordOutput("Indexer/FuelDetected", true);
        }
        
        lastSensorState = currentState;
        
        // Log sensor distance
        Logger.recordOutput("Indexer/SensorDistance", sensor.getRange());
        Logger.recordOutput("Indexer/HasFuel", currentState);
    }
    
    /**
     * Checks if fuel is currently in the sensor beam.
     * 
     * @return true if fuel is detected
     */
    public boolean hasFuel() {
        double distance = sensor.getRange();
        return distance < DETECTION_THRESHOLD_MM && distance > 0;
    }
    
    /**
     * Gets the current fuel count.
     * 
     * @return Number of fuel pieces counted
     */
    public int getFuelCount() {
        return fuelCount;
    }
    
    /**
     * Resets the fuel count to zero.
     */
    public void resetCount() {
        fuelCount = 0;
        Logger.recordOutput("Indexer/FuelCount", fuelCount);
    }
    
    /**
     * Checks if the hopper is full (max capacity reached).
     * 
     * @param maxCapacity Maximum fuel capacity
     * @return true if at max capacity
     */
    public boolean isFull(int maxCapacity) {
        return fuelCount >= maxCapacity;
    }
    
    /**
     * Checks if the hopper is empty.
     * 
     * @return true if no fuel counted
     */
    public boolean isEmpty() {
        return fuelCount == 0;
    }
}