package frc.robot.subsystems.Wrist;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulation implementation of WristIO.
 * 
 * This class simulates the wrist 4-bar linkage physics for testing and development
 * without physical hardware. Uses SingleJointedArmSim with gravity and ProfiledPIDController
 * to replicate CTRE MotionMagic position control behavior.
 * Compatible with AdvantageScope visualization.
 */
public class WristIOSim implements WristIO {
    
    // Arm simulation model with gravity
    private final SingleJointedArmSim armSim;
    
    // ProfiledPIDController for MotionMagic simulation
    private final ProfiledPIDController pid;
    
    // Feedforward for gravity compensation
    private final ArmFeedforward feedforward;
    
    // State tracking
    private double appliedVolts = 0.0;
    private double targetPosition = 0.0;
    private boolean closedLoopMode = false;
    
    public WristIOSim() {
        // Create arm simulation with gravity
        armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            WristConstants.GEAR_RATIO,
            WristConstants.SIM_MOI,
            WristConstants.SIM_ARM_LENGTH,
            WristConstants.SIM_MIN_ANGLE_RAD,
            WristConstants.SIM_MAX_ANGLE_RAD,
            true, // Simulate gravity
            WristConstants.SIM_MIN_ANGLE_RAD // Starting angle
        );
        
        // Initialize ProfiledPIDController with MotionMagic constraints
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(
            WristConstants.CRUISE_VELOCITY * 2 * Math.PI, // Convert rps to rad/s
            WristConstants.ACCELERATION * 2 * Math.PI  // Convert rps^2 to rad/s^2
        );
        
        pid = new ProfiledPIDController(
            WristConstants.kP,
            WristConstants.kI,
            WristConstants.kD,
            constraints
        );
        
        // Initialize feedforward for gravity compensation
        feedforward = new ArmFeedforward(
            WristConstants.kS,
            WristConstants.kG,
            WristConstants.kV * 2 * Math.PI, // Convert to rad/s
            WristConstants.kA * 2 * Math.PI  // Convert to rad/s^2
        );
    }
    
    @Override
    public void updateInputs(WristIOInputs inputs) {
        // Update simulation physics (20ms loop time)
        if (closedLoopMode) {
            // Convert target from rotations to radians
            double targetRadians = targetPosition * 2 * Math.PI;
            
            // Clamp target to limits
            targetRadians = Math.max(WristConstants.SIM_MIN_ANGLE_RAD,
                           Math.min(WristConstants.SIM_MAX_ANGLE_RAD, targetRadians));
            
            // Calculate profiled PID output
            double pidOutput = pid.calculate(armSim.getAngleRads(), targetRadians);
            
            // Calculate feedforward
            double ff = feedforward.calculate(
                pid.getSetpoint().position,
                pid.getSetpoint().velocity
            );
            
            appliedVolts = pidOutput + ff;
        }
        
        // Clamp voltage to realistic limits
        appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
        
        // Update simulation
        armSim.setInputVoltage(appliedVolts);
        armSim.update(0.02);
        
        // Update inputs (convert radians back to rotations)
        inputs.positionRotations = armSim.getAngleRads() / (2 * Math.PI);
        inputs.velocityRPS = armSim.getVelocityRadPerSec() / (2 * Math.PI);
        inputs.appliedVolts = appliedVolts;
        
        // Simulate current draw
        inputs.supplyCurrentAmps = Math.abs(appliedVolts) * 5.0; // Simplified
        inputs.statorCurrentAmps = armSim.getCurrentDrawAmps();
        
        // Simulate temperature
        inputs.tempCelsius = 25.0 + inputs.statorCurrentAmps * 0.5;
        
        inputs.targetPositionRotations = targetPosition;
    }
    
    @Override
    public void setPosition(double positionRotations) {
        closedLoopMode = true;
        // Clamp position to limits
        targetPosition = Math.max(WristConstants.MIN_ANGLE_ROTATIONS,
                        Math.min(WristConstants.MAX_ANGLE_ROTATIONS, positionRotations));
    }
    
    @Override
    public void zeroEncoder() {
        // In simulation, reset the arm to the current angle as "zero"
        // This simulates zeroing the encoder at a known position
        targetPosition = 0.0;
        // Note: In real sim, we'd need to track an offset, but for simplicity we reset the sim
        armSim.setState(WristConstants.SIM_MIN_ANGLE_RAD, 0.0);
    }
    
    @Override
    public void stop() {
        closedLoopMode = false;
        appliedVolts = 0.0;
        targetPosition = armSim.getAngleRads() / (2 * Math.PI);
        pid.reset(armSim.getAngleRads());
    }
    
    @Override
    public void setVoltage(double volts) {
        closedLoopMode = false;
        appliedVolts = volts;
        pid.reset(armSim.getAngleRads());
    }
}