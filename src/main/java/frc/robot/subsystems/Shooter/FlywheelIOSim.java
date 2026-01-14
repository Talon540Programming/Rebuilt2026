package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO {
    
    // Constants for simulation - 2x X60 motors
    private static final double kGearing = 1.0; // Direct drive flywheel
    private static final double kMOI = 0.01; // kg*m^2 - flywheel with Colson wheels
    
    // Create the plant (linear system model) first - WPILib 2026 API
    private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX60(2), // 2 motors
        kMOI,
        kGearing
    );
    
    // FlywheelSim constructor in 2026: (LinearSystem plant, DCMotor gearbox)
    private final FlywheelSim sim = new FlywheelSim(
        plant,
        DCMotor.getKrakenX60(2)
    );
    
    private double appliedVolts = 0.0;
    private double targetVelocityRPM = 0.0;
    
    // Simple velocity control gains for simulation
    private static final double kSimP = 0.01;
    private static final double kSimFF = 0.0018; // Feedforward: volts per RPM
    
    // Velocity tolerance for "at setpoint" check (RPM)
    private static final double kVelocityTolerance = 50.0;
    
    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        sim.update(0.02);
        
        inputs.velocityRotPerSec = sim.getAngularVelocityRPM() / 60.0;
        inputs.velocityRPM = sim.getAngularVelocityRPM();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.tempCelsius = 25.0;
        inputs.targetVelocityRPM = targetVelocityRPM;
        inputs.atSetpoint = Math.abs(inputs.velocityRPM - targetVelocityRPM) < kVelocityTolerance;
    }
    
    @Override
    public void setVelocity(double velocityRPM) {
        targetVelocityRPM = velocityRPM;
        
        // Simple P + FF control for simulation
        double currentRPM = sim.getAngularVelocityRPM();
        double error = velocityRPM - currentRPM;
        appliedVolts = (kSimP * error) + (kSimFF * velocityRPM);
        appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
        
        sim.setInputVoltage(appliedVolts);
    }
    
    @Override
    public void setDutyCycle(double dutyCycle) {
        targetVelocityRPM = 0.0;
        appliedVolts = dutyCycle * 12.0;
        sim.setInputVoltage(appliedVolts);
    }
    
    @Override
    public void stop() {
        targetVelocityRPM = 0.0;
        appliedVolts = 0.0;
        sim.setInputVoltage(0.0);
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        // Simulation doesn't need brake mode
    }
}