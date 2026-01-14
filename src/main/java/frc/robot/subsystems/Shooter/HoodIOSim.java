package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ShooterConstants;

public class HoodIOSim implements HoodIO {
    
    // Simulate as a single jointed arm (hood pivots up/down)
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        DCMotor.getKrakenX44(1),
        50.0, // Gear ratio
        0.1,  // MOI kg*m^2 - light hood mechanism
        0.15, // Arm length meters
        ShooterConstants.kHoodMinAngle, // Min angle
        ShooterConstants.kHoodMaxAngle, // Max angle
        false, // Don't simulate gravity (rack and pinion holds position)
        ShooterConstants.kHoodMinAngle // Starting angle
    );
    
    private double appliedVolts = 0.0;
    private double targetPositionRadians = ShooterConstants.kHoodMinAngle;
    
    // Simple position control gains for simulation
    private static final double kSimP = 5.0;
    
    // Position tolerance (radians)
    private static final double kPositionTolerance = Math.toRadians(2.0);
    
    @Override
    public void updateInputs(HoodIOInputs inputs) {
        sim.update(0.02);
        
        inputs.positionRotations = sim.getAngleRads() / (2 * Math.PI);
        inputs.positionRadians = sim.getAngleRads();
        inputs.velocityRotPerSec = sim.getVelocityRadPerSec() / (2 * Math.PI);
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.tempCelsius = 25.0;
        inputs.targetPositionRadians = targetPositionRadians;
        inputs.atSetpoint = Math.abs(inputs.positionRadians - targetPositionRadians) < kPositionTolerance;
    }
    
    @Override
    public void setPosition(double positionRadians) {
        // Clamp to valid range
        positionRadians = Math.max(ShooterConstants.kHoodMinAngle, 
                         Math.min(ShooterConstants.kHoodMaxAngle, positionRadians));
        targetPositionRadians = positionRadians;
        
        // Simple P control for simulation
        double error = positionRadians - sim.getAngleRads();
        appliedVolts = kSimP * error;
        appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
        
        sim.setInputVoltage(appliedVolts);
    }
    
    @Override
    public void setDutyCycle(double dutyCycle) {
        appliedVolts = dutyCycle * 12.0;
        sim.setInputVoltage(appliedVolts);
    }
    
    @Override
    public void stop() {
        appliedVolts = 0.0;
        sim.setInputVoltage(0.0);
    }
    
    @Override
    public void zeroEncoder() {
        // In simulation, reset the sim state to min angle
        sim.setState(ShooterConstants.kHoodMinAngle, 0.0);
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        // Simulation doesn't need brake mode
    }
}