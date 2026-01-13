package frc.robot.subsystems.Index;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IndexIOSim implements IndexIO {
    
    // Constants for simulation
    private static final double kGearing = 3.0; // Gear ratio estimate
    private static final double kMOI = 0.005; // kg*m^2 - small belts/rollers
    
    // Create the plant (linear system model) first - WPILib 2026 API
    private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX44(1),
        kMOI,
        kGearing
    );
    
    // FlywheelSim constructor in 2026: (LinearSystem plant, DCMotor gearbox)
    private final FlywheelSim sim = new FlywheelSim(
        plant,
        DCMotor.getKrakenX44(1)
    );
    
    private double appliedVolts = 0.0;
    private boolean simulatedGamePiece = false;
    private double simulatedDistance = 1.0; // Default to no game piece (far away)
    
    @Override
    public void updateInputs(IndexIOInputs inputs) {
        sim.update(0.02);
        
        inputs.velocityRotPerSec = sim.getAngularVelocityRPM() / 60.0;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.tempCelsius = 25.0;
        
        // Simulate CANRange data
        inputs.distanceMeters = simulatedDistance;
        inputs.hasGamePiece = simulatedGamePiece;
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
    
    // ==================== SIMULATION METHODS ====================
    
    /** For testing - simulate a game piece being present at the CANRange */
    public void setSimulatedGamePiece(boolean hasGamePiece) {
        this.simulatedGamePiece = hasGamePiece;
        this.simulatedDistance = hasGamePiece ? 0.05 : 1.0; // 5cm if present, 1m if not
    }
    
    /** For testing - set the simulated distance reading */
    public void setSimulatedDistance(double distanceMeters) {
        this.simulatedDistance = distanceMeters;
        this.simulatedGamePiece = distanceMeters < 0.15 && distanceMeters > 0.01;
    }
}