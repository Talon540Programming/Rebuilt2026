package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Shooter.ShooterConstants;

/**
 * Simulation implementation of IndexerIO.
 * 
 * This class simulates the indexer belt/roller physics for testing and development
 * without physical hardware. Uses TrapezoidProfile to replicate CTRE MotionMagic
 * velocity control with smooth acceleration and jerk limiting.
 * Compatible with AdvantageScope visualization.
 */
public class IndexerIOSim implements IndexerIO {
    
    // Indexer simulation model
    private final FlywheelSim indexerSim;
    
    // PID controller for closed-loop simulation
    private final PIDController pid;
    
    // TrapezoidProfile for MotionMagic simulation
    private TrapezoidProfile.Constraints constraints;
    private TrapezoidProfile.State setpointState;
    private TrapezoidProfile.State goalState;
    
    // State tracking
    private double appliedVolts = 0.0;
    private double targetVelocity = 0.0;
    private boolean closedLoopMode = false;
    
    public IndexerIOSim() {
        // Create indexer simulation using X60 FOC as approximation for X44

        LinearSystem<N1, N1, N1> indexFlyweehlPlant =
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                ShooterConstants.TOP_FLYWHEEL_GEAR_RATIO,
                ShooterConstants.SIM_TOP_MOI
            );

        indexerSim = new FlywheelSim(indexFlyweehlPlant, DCMotor.getKrakenX60(1), ShooterConstants.TOP_FLYWHEEL_GEAR_RATIO);

        
        // Initialize PID controller for simulation
        pid = new PIDController(
            IndexerConstants.kP,
            IndexerConstants.kI,
            IndexerConstants.kD
        );
        
        // Initialize MotionMagic constraints
        // For velocity control, we profile the velocity itself
        constraints = new TrapezoidProfile.Constraints(
            IndexerConstants.ACCELERATION, // Max velocity change rate (acceleration)
            IndexerConstants.JERK // Max acceleration change rate (jerk)
        );
        
        setpointState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
    }
    
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        // Update simulation physics (20ms loop time)
        if (closedLoopMode) {
            // Update goal state with target velocity
            goalState = new TrapezoidProfile.State(targetVelocity, 0);

            
            // Generate motion profile to smoothly reach target velocity
            TrapezoidProfile profile = new TrapezoidProfile(constraints);
            
            setpointState = profile.calculate(0.02, goalState, goalState); // Calculate next setpoint
            
            // Use the profiled velocity as our control setpoint
            double profiledVelocity = setpointState.position;
            double profiledAcceleration = setpointState.velocity;
            
            // Closed-loop control: PID + feedforward
            double ff = profiledVelocity * IndexerConstants.kV 
                + Math.signum(profiledVelocity) * IndexerConstants.kS
                + profiledAcceleration * IndexerConstants.kA;
            
            double pidOutput = pid.calculate(getVelocityRPS(), profiledVelocity);
            
            appliedVolts = pidOutput + ff;
        }
        
        // Clamp voltage to realistic limits
        appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
        
        // Update simulation
        indexerSim.setInputVoltage(appliedVolts);
        indexerSim.update(0.02);
        
        // Update inputs
        inputs.velocityRPS = getVelocityRPS();
        inputs.appliedVolts = appliedVolts;
        
        // Simulate current draw
        inputs.supplyCurrentAmps = Math.abs(appliedVolts) * 5.0; // Simplified
        inputs.statorCurrentAmps = indexerSim.getCurrentDrawAmps();
        
        // Simulate temperature
        inputs.tempCelsius = 25.0 + inputs.statorCurrentAmps * 0.5;
        
        inputs.targetVelocityRPS = targetVelocity;
    }
    
    @Override
    public void setVelocity(double velocityRPS) {
        closedLoopMode = true;
        targetVelocity = velocityRPS;
    }
    
    @Override
    public void stop() {
        closedLoopMode = false;
        appliedVolts = 0.0;
        targetVelocity = 0.0;
        setpointState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        pid.reset();
    }
    
    @Override
    public void setVoltage(double volts) {
        closedLoopMode = false;
        appliedVolts = volts;
        targetVelocity = 0.0;
        setpointState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        pid.reset();
    }
    
    /**
     * Gets the simulated indexer velocity in rotations per second.
     * Converts from rad/s to rps.
     */
    private double getVelocityRPS() {
        return indexerSim.getAngularVelocityRPM() / 60.0;
    }
}