package frc.robot.subsystems.Intake.Roller;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class RollerIOSim implements RollerIO {
    
    // Constants for simulation
    private static final double kGearing = 3.0; // Gear ratio
    private static final double kMOI = 0.01; // kg*m^2
    
    // Create the plant (linear system model) first - using 2 Kraken X60s
    private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX60(2),
        kMOI,
        kGearing
    );
    
    // FlywheelSim constructor in 2026: (LinearSystem plant, DCMotor gearbox, double... measurementStdDevs)
    // Using 2 Kraken X60s now
    private final FlywheelSim sim = new FlywheelSim(
        plant,
        DCMotor.getKrakenX60(2)
    );
    
    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        sim.update(0.02);
        
        inputs.velocityRotPerSec = sim.getAngularVelocityRPM() / 60.0;
        inputs.appliedVolts = appliedVolts;
        
        // Split current between two simulated motors
        double totalCurrent = sim.getCurrentDrawAmps();
        inputs.currentAmps = totalCurrent / 2.0;
        inputs.followerCurrentAmps = totalCurrent / 2.0;
        
        inputs.tempCelsius = 25.0;
        inputs.followerTempCelsius = 25.0;
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