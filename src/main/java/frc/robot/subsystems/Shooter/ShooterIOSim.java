package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/**
 * Simulation implementation of ShooterIO.
 * 
 * This class simulates the shooter flywheel physics for testing and development
 * without physical hardware. Uses a leader-follower model where the bottom motor
 * runs velocity control and the top motor mechanically follows.
 * Compatible with AdvantageScope visualization.
 */
public class ShooterIOSim implements ShooterIO {
    
    // Flywheel simulation models
    private final FlywheelSim bottomFlywheelSim;
    private final FlywheelSim topFlywheelSim;
    
    // PID controller for leader motor closed-loop simulation
    private final PIDController bottomPID;
    
    // State tracking
    private double bottomAppliedVolts = 0.0;
    private double topAppliedVolts = 0.0;
    private double targetBottomVelocity = 0.0;
    private boolean closedLoopMode = false;
    
    public ShooterIOSim() {
        // Create flywheel simulation models using X60 Kraken motor
        // Bottom flywheel: single X60 (leader motor)
        

        LinearSystem<N1, N1, N1> flyWheelPlant = 
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                ShooterConstants.BOTTOM_FLYWHEEL_GEAR_RATIO,
                ShooterConstants.SIM_TOP_MOI
            );

        bottomFlywheelSim = new FlywheelSim(flyWheelPlant, DCMotor.getKrakenX60(1), ShooterConstants.BOTTOM_FLYWHEEL_GEAR_RATIO);
        
        // Top flywheel: single X60 (follower motor)
        // Smaller wheel = less MOI, spins 3x faster due to gear ratio

        LinearSystem<N1, N1, N1> topFlyWheelPlant =
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                ShooterConstants.TOP_FLYWHEEL_GEAR_RATIO,
                ShooterConstants.SIM_TOP_MOI
            );

        topFlywheelSim = new FlywheelSim(topFlyWheelPlant, DCMotor.getKrakenX60(1), ShooterConstants.TOP_FLYWHEEL_GEAR_RATIO);
        
        // Initialize PID controller for leader motor simulation
        bottomPID = new PIDController(
            ShooterConstants.kP,
            ShooterConstants.kI,
            ShooterConstants.kD
        );
    }
    
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Update simulation physics (20ms loop time)
        if (closedLoopMode) {
            // Closed-loop control on leader motor: PID + feedforward
            double bottomFF = targetBottomVelocity * ShooterConstants.kV 
                + Math.signum(targetBottomVelocity) * ShooterConstants.kS;
            
            double bottomPIDOutput = bottomPID.calculate(getBottomVelocityRPS(), targetBottomVelocity);
            
            bottomAppliedVolts = bottomPIDOutput + bottomFF;
            
            // Follower motor receives the same voltage as leader (simulating mechanical following)
            topAppliedVolts = bottomAppliedVolts;
        }
        // In open-loop mode, both motors get the same voltage (set by setVoltage)
        
        // Clamp voltages to realistic limits
        bottomAppliedVolts = Math.max(-12.0, Math.min(12.0, bottomAppliedVolts));
        topAppliedVolts = Math.max(-12.0, Math.min(12.0, topAppliedVolts));
        
        // Update simulations with applied voltages
        bottomFlywheelSim.setInputVoltage(bottomAppliedVolts);
        topFlywheelSim.setInputVoltage(topAppliedVolts);
        
        bottomFlywheelSim.update(0.02);
        topFlywheelSim.update(0.02);
        
        // Update inputs
        inputs.bottomVelocityRPS = getBottomVelocityRPS();
        inputs.topVelocityRPS = getTopVelocityRPS();
        inputs.bottomAppliedVolts = bottomAppliedVolts;
        inputs.topAppliedVolts = topAppliedVolts;
        
        // Simulate current draw (approximate based on simulation)
        inputs.bottomSupplyCurrentAmps = Math.abs(bottomAppliedVolts) * 5.0; // Simplified
        inputs.topSupplyCurrentAmps = Math.abs(topAppliedVolts) * 5.0;
        inputs.bottomStatorCurrentAmps = bottomFlywheelSim.getCurrentDrawAmps();
        inputs.topStatorCurrentAmps = topFlywheelSim.getCurrentDrawAmps();
        
        // Simulate temperature (slowly increases with current draw)
        inputs.bottomTempCelsius = 25.0 + inputs.bottomStatorCurrentAmps * 0.5;
        inputs.topTempCelsius = 25.0 + inputs.topStatorCurrentAmps * 0.5;
        
        // Update target velocities
        inputs.targetBottomVelocityRPS = targetBottomVelocity;
        inputs.targetTopVelocityRPS = targetBottomVelocity * 3.0; // Top spins 3x faster
    }
    
    @Override
    public void setVelocity(double bottomVelocityRPS) {
        closedLoopMode = true;
        targetBottomVelocity = bottomVelocityRPS;
        // Top velocity is automatically 3x due to mechanical following (different gear ratio)
    }
    
    @Override
    public void stop() {
        closedLoopMode = false;
        bottomAppliedVolts = 0.0;
        topAppliedVolts = 0.0;
        targetBottomVelocity = 0.0;
        bottomPID.reset();
    }
    
    @Override
    public void setVoltage(double volts) {
        closedLoopMode = false;
        bottomAppliedVolts = volts;
        topAppliedVolts = volts; // Follower gets same voltage as leader
        targetBottomVelocity = 0.0;
        bottomPID.reset();
    }
    
    /**
     * Gets the simulated bottom flywheel velocity in rotations per second.
     * Converts from rad/s to rps.
     */
    private double getBottomVelocityRPS() {
        return bottomFlywheelSim.getAngularVelocityRPM() / 60.0;
    }
    
    /**
     * Gets the simulated top flywheel velocity in rotations per second.
     * Top wheel spins 3x faster than bottom due to smaller diameter (gear ratio).
     * Converts from rad/s to rps.
     */
    private double getTopVelocityRPS() {
        return topFlywheelSim.getAngularVelocityRPM() / 60.0;
    }
}