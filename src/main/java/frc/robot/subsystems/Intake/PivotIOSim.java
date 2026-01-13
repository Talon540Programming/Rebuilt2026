package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class PivotIOSim implements PivotIO {
    
    // Simulate as a single jointed arm
    private final SingleJointedArmSim sim = new SingleJointedArmSim(
        DCMotor.getKrakenX44(1),
        50.0, // Gear ratio estimate
        0.5,  // MOI kg*m^2
        0.3,  // Arm length meters
        Math.toRadians(-10), // Min angle (stowed)
        Math.toRadians(100), // Max angle (deployed)
        true, // Simulate gravity
        Math.toRadians(0) // Starting angle
    );
    
    private double appliedVolts = 0.0;
    
    @Override
    public void updateInputs(PivotIOInputs inputs) {
        sim.update(0.02);
        
        inputs.positionRotations = Math.toDegrees(sim.getAngleRads()) / 360.0;
        inputs.velocityRotPerSec = Math.toDegrees(sim.getVelocityRadPerSec()) / 360.0;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.tempCelsius = 25.0;
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
    public void setBrakeMode(boolean brake) {
        // Simulation doesn't need brake mode
    }
}