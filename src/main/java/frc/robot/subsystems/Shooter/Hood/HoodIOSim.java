package frc.robot.subsystems.Shooter.Hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.ShootingConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodIOSim implements HoodIO {
    
    // Simulate as a single jointed arm (hood pivots up/down)
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        DCMotor.getKrakenX44(1),
        50.0, // Gear ratio
        0.1,  // MOI kg*m^2 - light hood mechanism
        0.15, // Arm length meters
        ShooterConstants.hoodMinPosRot.get(), // Min angle
        ShooterConstants.hoodMaxPosRot.get(), // Max angle
        false, // Don't simulate gravity (rack and pinion holds position)
        ShooterConstants.hoodMinPosRot.get() // Starting angle
    );

    private boolean closedLoop = false;
    private double appliedVolts = 0.0;
    private double targetPositionRadians = ShooterConstants.hoodMinPosRot.get();
    
    // Simple position control gains for simulation
    private static final double kSimP = 5.0;
    
    // Position tolerance (radians)
    private static final double kPositionTolerance = Math.toRadians(2.0);
    
    @Override
    public void updateInputs(HoodIOInputs inputs) {
        // Run closed-loop control if enabled
        if (closedLoop) {
            double error = targetPositionRadians - sim.getAngleRads();
            appliedVolts = kSimP * error;
            appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
            sim.setInputVoltage(appliedVolts);
        }
        
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
        positionRadians = MathUtil.clamp(positionRadians, ShooterConstants.hoodMinPosRot.get(), ShooterConstants.hoodMaxPosRot.get());
        targetPositionRadians = positionRadians;
        closedLoop = true;
    }
    
    @Override
    public void setDutyCycle(double dutyCycle) {
        closedLoop = false;
        appliedVolts = dutyCycle * 12.0;
        sim.setInputVoltage(appliedVolts);
    }
    
    @Override
    public void stop() {
        closedLoop = false;
        appliedVolts = 0.0;
        sim.setInputVoltage(0.0);
    }
    
    @Override
    public void zeroEncoder() {
        // In simulation, reset the sim state to min angle
        sim.setState(ShooterConstants.hoodMinPosRot.get(), 0.0);
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        // Simulation doesn't need brake mode
    }
}