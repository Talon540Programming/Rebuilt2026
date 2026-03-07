package frc.robot.subsystems.Indexer;

public interface IndexerIO {
    
    public class IndexerIOInputs {

        double appliedVoltage = 0.0;
        double currentAmps = 0.0;
        double temp = 0.0;
        double velocityRotPerSec = 0.0;
        boolean connected = false;

    }

    default void updateIndexerIOInputs(IndexerIOInputs inputs) {};

    default void setDutyCycle (double dutyCycle) {}

    default void stop () {}

    default void breakMode (boolean breakMode) {}

}
