package frc.robot.subsystems.Intake.Roller;

import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
    
    @AutoLog
    public static class RollerIOInputs {
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
        public boolean hasGamePiece = false;
    }
    
    /** Updates the set of loggable inputs */
    default void updateInputs(RollerIOInputs inputs) {}
    
    /** Run the rollers at the specified duty cycle (-1 to 1) */
    default void setDutyCycle(double dutyCycle) {}
    
    /** Stop the roller motor */
    default void stop() {}
    
    /** Set the neutral mode (brake or coast) */
    default void setBrakeMode(boolean brake) {}
}