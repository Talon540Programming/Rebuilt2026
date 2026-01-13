package frc.robot.subsystems.Intake;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class RollerSim implements RollerIO {
    
    // Constants for simulation
    private static final double kGearing = 3.0; // Gear ratio
    private static final double kMOI = 0.01; // kg*m^2
    
    // Create the plant (linear system model) first
    private final LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(
        DCMotor.getKrakenX44(1),
        kMOI,
        kGearing
    );
    
    // FlywheelSim constructor in 2026: (LinearSystem plant, DCMotor gearbox, double... measurementStdDevs)
    private final FlywheelSim sim = new FlywheelSim(
        plant,
        DCMotor.getKrakenX44(1)
    );
    
    private double appliedVolts = 0.0;
    private boolean isRunning = false;
    private boolean simulatedGamePiece = false;
    
    @Override
    public void updateInputs(RollerIOInputs inputs) {
        sim.update(0.02);
        
        inputs.velocityRotPerSec = sim.getAngularVelocityRPM() / 60.0;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.tempCelsius = 25.0;
        
        // Simulate game piece detection
        inputs.hasGamePiece = isRunning && simulatedGamePiece;
    }
    
    @Override
    public void setDutyCycle(double dutyCycle) {
        isRunning = Math.abs(dutyCycle) > 0.01;
        appliedVolts = dutyCycle * 12.0;
        sim.setInputVoltage(appliedVolts);
    }
    
    @Override
    public void stop() {
        isRunning = false;
        appliedVolts = 0.0;
        sim.setInputVoltage(0.0);
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        // Simulation doesn't need brake mode
    }
    
    /** For testing - simulate a game piece being present */
    public void setSimulatedGamePiece(boolean hasGamePiece) {
        this.simulatedGamePiece = hasGamePiece;
    }
}