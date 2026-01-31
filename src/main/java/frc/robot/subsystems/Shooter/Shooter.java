package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;


/**
 * Shooter subsystem for launching FUEL into the HUB.
 * 
 * The shooter consists of two X60 Kraken motors driving flywheels:
 * - Bottom flywheel (leader): larger diameter, controlled directly
 * - Top flywheel (follower): smaller diameter (3x faster), follows bottom
 * 
 * Both flywheels use colsons (bottom) and high durometer compliant wheels (top).
 * The shooter runs on velocity PID control for consistent shooting performance.
 */
public class Shooter extends SubsystemBase {
    
    private final ShooterIO io;
    private final ShooterIOInputs inputs = new ShooterIOInputs();
    
    /**
     * Shooter state machine states
     */
    public enum ShooterState {
        /** Shooter is not spinning */
        NOT_SHOOTING,
        /** Shooter is spinning at target velocity */
        SHOOTING
    }
    
    private ShooterState currentState = ShooterState.NOT_SHOOTING;
    
    /**
     * Creates a new Shooter subsystem.
     * 
     * @param io The hardware interface implementation (real or simulated)
     */
    public Shooter(ShooterIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        // Update and log inputs
        io.updateInputs(inputs);
              
        // Log state machine info
        Logger.recordOutput("Shooter/CurrentState", currentState.toString());
        Logger.recordOutput("Shooter/AtTargetVelocity", isAtTargetVelocity());
        Logger.recordOutput("Shooter/VelocityError", getVelocityError());
        
        // Debug logging
        Logger.recordOutput("Shooter/Debug/BottomVelocityRPS", inputs.bottomVelocityRPS);
        Logger.recordOutput("Shooter/Debug/TargetBottomVelocityRPS", inputs.targetBottomVelocityRPS);
        Logger.recordOutput("Shooter/Debug/BottomAppliedVolts", inputs.bottomAppliedVolts);
        Logger.recordOutput("Shooter/Debug/BottomStatorCurrent", inputs.bottomStatorCurrentAmps);
    }
    
    /**
     * Sets the shooter to a target velocity using closed-loop control.
     * The bottom flywheel will spin at the specified velocity, while the top
     * flywheel automatically spins 3x faster due to size difference.
     * Sets state to SHOOTING.
     * 
     * @param bottomVelocityRPS Target velocity for the bottom flywheel in rotations per second
     */
    public void setVelocity(double bottomVelocityRPS) {
        currentState = ShooterState.SHOOTING;
        io.setVelocity(bottomVelocityRPS);
        
        // Debug log
        Logger.recordOutput("Shooter/Debug/SetVelocityCalled", bottomVelocityRPS);
    }
    
    /**
     * Stops both flywheel motors.
     * Sets motor output to neutral and state to NOT_SHOOTING.
     */
    public void stop() {
        currentState = ShooterState.NOT_SHOOTING;
        io.stop();
        
        // Debug log
        Logger.recordOutput("Shooter/Debug/StopCalled", true);
    }
    
    /**
     * Sets the voltage output for both flywheels (open-loop control).
     * Primarily used for characterization and testing.
     * 
     * @param volts Voltage to apply (-12 to +12V)
     */
    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }
    
    /**
     * Gets the current state of the shooter.
     * 
     * @return Current ShooterState
     */
    public ShooterState getCurrentState() {
        return currentState;
    }
    
    /**
     * Checks if both flywheels are at their target velocity within tolerance.
     * Returns false if target is zero (not shooting).
     * 
     * @return true if both flywheels are within tolerance of their targets
     */
    public boolean isAtTargetVelocity() {
        // If target is 0, we're not trying to shoot - return false
        if (inputs.targetBottomVelocityRPS < 0.1) {
            return false;
        }
        
        double bottomError = Math.abs(inputs.bottomVelocityRPS - inputs.targetBottomVelocityRPS);
        double topError = Math.abs(inputs.topVelocityRPS - inputs.targetTopVelocityRPS);
        
        return bottomError < ShooterConstants.VELOCITY_TOLERANCE_RPS
            && topError < ShooterConstants.VELOCITY_TOLERANCE_RPS * 3.0; // Top wheel has 3x tolerance
    }
    
    /**
     * Gets the velocity error for the bottom flywheel.
     * 
     * @return Velocity error in rotations per second
     */
    public double getVelocityError() {
        return inputs.targetBottomVelocityRPS - inputs.bottomVelocityRPS;
    }
    
    /**
     * Gets the current velocity of the bottom flywheel.
     * 
     * @return Bottom flywheel velocity in rotations per second
     */
    public double getBottomVelocity() {
        return inputs.bottomVelocityRPS;
    }
    
    /**
     * Gets the current velocity of the top flywheel.
     * 
     * @return Top flywheel velocity in rotations per second
     */
    public double getTopVelocity() {
        return inputs.topVelocityRPS;
    }
    
    /**
     * Gets the target velocity of the bottom flywheel.
     * 
     * @return Target velocity in rotations per second
     */
    public double getTargetVelocity() {
        return inputs.targetBottomVelocityRPS;
    }
}