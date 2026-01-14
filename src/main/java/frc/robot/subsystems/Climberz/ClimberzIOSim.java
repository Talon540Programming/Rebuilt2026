package frc.robot.subsystems.Climberz;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ClimberzIOSim implements ClimberzIO {
    
    // Constants for simulation - Andymark climber with 2x X60
    private static final double kGearing = 25.0; // Typical climber gear ratio
    private static final double kMOI = 0.01;     // kg*m^2
    
    // Create the plant (2 motors on same mechanism)
    private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX60(2), // 2 motors
        kMOI,
        kGearing
    );
    
    private final FlywheelSim sim = new FlywheelSim(plant, DCMotor.getKrakenX60(2));
    
    private double appliedVolts = 0.0;
    private double positionRotations = 0.0;
    
    @Override
    public void updateInputs(ClimberzIOInputs inputs) {
        sim.update(0.02);
        
        // Integrate velocity to get position
        positionRotations += (sim.getAngularVelocityRPM() / 60.0) * 0.02;
        
        inputs.positionRotations = positionRotations;
        inputs.velocityRotPerSec = sim.getAngularVelocityRPM() / 60.0;
        inputs.appliedVolts = appliedVolts;
        // Split current between two motors
        inputs.leaderCurrentAmps = sim.getCurrentDrawAmps() / 2.0;
        inputs.followerCurrentAmps = sim.getCurrentDrawAmps() / 2.0;
        inputs.leaderTempCelsius = 25.0;
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