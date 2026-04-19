package frc.robot.subsystems.Shooter.Flywheel;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class FlywheelIOSim implements FlywheelIO {
    
    // Constants for simulation - 2x X60 motors
    private static final double kGearing = 1.0; // Direct drive flywheel
    private static final double kMOI = 0.01; // kg*m^2 - flywheel with Colson wheels
    // Shot velocity drop (percentage of current RPM lost per shot)
    private static final double kShotVelocityDropPercent = 0.10;
    
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
    
    // Velocity tolerance for "at setpoint" check (RPM)
    private static final double kVelocityTolerance = 50.0;
    
    public FlywheelIOSim() {
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        sim.setInputVoltage(appliedVolts);
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
    public void stop() {
        targetVelocityRPM = 0.0;
        appliedVolts = 0.0;
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        // Simulation doesn't need brake mode
    }
    
    @Override
    public void runVelocity(double velocityRPM) {
        targetVelocityRPM = velocityRPM;
        
        if (velocityRPM <= 0) {
            appliedVolts = 0.0;
            return;
        }
        
        // Simulate Motion Magic Velocity with feedforward + P control
        double currentRPM = sim.getAngularVelocityRPM();
        double targetRotPerSec = velocityRPM / 60.0;
        double currentRotPerSec = currentRPM / 60.0;
        
        // Feedforward: V = kS + kV * velocity
        double kS = ShooterConstants.flywheelkS.get();
        double kV = ShooterConstants.flywheelkV.get();
        double kP = ShooterConstants.flywheelkP.get();
        
        double feedforward = kS + kV * targetRotPerSec;
        
        // P control for error correction
        double error = targetRotPerSec - currentRotPerSec;
        double feedback = kP * error;
        
        // Total voltage
        appliedVolts = feedforward + feedback;
        
        // Clamp to valid range
        appliedVolts = Math.max(0.0, Math.min(12.0, appliedVolts));
    }

    @Override
    public void simulateShot() {
        double currentRPM = sim.getAngularVelocityRPM();
        double dropRPM = currentRPM * kShotVelocityDropPercent;
        double newRPM = currentRPM - dropRPM;
        
        // Convert RPM to rad/s and set the sim state
        double newAngularVelocity = newRPM * 2.0 * Math.PI / 60.0;
        sim.setAngularVelocity(newAngularVelocity);
    }
}
