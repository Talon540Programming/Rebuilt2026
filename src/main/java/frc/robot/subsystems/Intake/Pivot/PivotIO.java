package frc.robot.subsystems.Intake.Pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    
    @AutoLog
    public static class PivotIOInputs {
        public double positionRotations = 0.0;
        public double velocityRotPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
    }
    
    /** Updates the set of loggable inputs */
    default void updateInputs(PivotIOInputs inputs) {}
    
    /** Run the pivot at the specified duty cycle (-1 to 1) */
    default void setDutyCycle(double dutyCycle) {}
    
    /** Stop the pivot motor */
    default void stop() {}
    
    /** Set the neutral mode (brake or coast) */
    default void setBrakeMode(boolean brake) {}

    /** Run closed-loop Motion Magic to a position (mechanism rotations). */
    default void runMotionMagicPosition(double positionRotations, double feedForwardVolts) {}

    /** Sets the current mechanism position (in mechanism rotations). Used for zeroing. */
    default void setPosition(double positionRotations) {}


}