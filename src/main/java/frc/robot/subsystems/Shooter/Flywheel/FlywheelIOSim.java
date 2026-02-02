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
    private double torqueCurrentAmps = 0.0;
    private boolean closedLoop = false;
    private double targetVelocityRPM = 0.0;
    @SuppressWarnings("unused")
    private FlywheelControlMode currentMode = FlywheelControlMode.COAST;
    
    // Simple velocity control gains for simulation
    private static final double kSimFF = 0.0018; // Feedforward: volts per RPM
    
    // Velocity tolerance for "at setpoint" check (RPM)
    private static final double kVelocityTolerance = 50.0;
    
    // Motor model for torque current simulation
    private static final DCMotor kMotorModel = DCMotor.getKrakenX60(2);

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        double currentRPM = sim.getAngularVelocityRPM();
        
        if (closedLoop) {
            double error = targetVelocityRPM - currentRPM;
            double simP = ShooterConstants.flywheelkP.get();
            appliedVolts = (simP * error) + (kSimFF * targetVelocityRPM);
            appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
        }
        
        sim.setInputVoltage(appliedVolts);
        sim.update(0.02);
        
        inputs.velocityRotPerSec = sim.getAngularVelocityRPM() / 60.0;
        inputs.velocityRPM = sim.getAngularVelocityRPM();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.torqueCurrentAmps = torqueCurrentAmps;
        inputs.tempCelsius = 25.0;
        inputs.targetVelocityRPM = targetVelocityRPM;
        inputs.atSetpoint = Math.abs(inputs.velocityRPM - targetVelocityRPM) < kVelocityTolerance;
    }
    
   @Override
    public void setVelocity(double velocityRPM) {
        targetVelocityRPM = velocityRPM;
        closedLoop = true;
    }
    
    @Override
    public void setDutyCycle(double dutyCycle) {
        closedLoop = false;
        targetVelocityRPM = 0.0;
        appliedVolts = dutyCycle * 12.0;
        sim.setInputVoltage(appliedVolts);
    }
    
    @Override
    public void stop() {
        closedLoop = false;
        targetVelocityRPM = 0.0;
        appliedVolts = 0.0;
        sim.setInputVoltage(0.0);
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        // Simulation doesn't need brake mode
    }
    
    @Override
    public void runBangBang(double velocityRPM, FlywheelControlMode mode) {
        targetVelocityRPM = velocityRPM;
        currentMode = mode;
        closedLoop = false;
        
        double currentRPM = sim.getAngularVelocityRPM();
        double tolerance = ShooterConstants.flywheelVelToleranceRPM.get();
        
        switch (mode) {
            case COAST:
                appliedVolts = 0.0;
                torqueCurrentAmps = 0.0;
                break;
                
            case DUTY_CYCLE_BANG_BANG:
                // Full voltage if below target (with deadband)
                if (currentRPM < velocityRPM - tolerance) {
                    appliedVolts = 12.0;
                    torqueCurrentAmps = 0.0;
                } else if (currentRPM > velocityRPM + tolerance) {
                    appliedVolts = 0.0;
                    torqueCurrentAmps = 0.0;
                }
                // In deadband: maintain previous output (hysteresis)
                break;
                
            case TORQUE_CURRENT_BANG_BANG:
                // Simulate torque current control (with deadband)
                if (currentRPM < velocityRPM - tolerance) {
                    torqueCurrentAmps = ShooterConstants.flywheelBangBangTorqueCurrent.get();
                    // Convert torque current to voltage: V = I * R + omega * Kv
                    double omegaRadPerSec = currentRPM * 2.0 * Math.PI / 60.0;
                    appliedVolts = (torqueCurrentAmps * kMotorModel.rOhms) 
                        + (omegaRadPerSec / kMotorModel.KvRadPerSecPerVolt);
                    appliedVolts = Math.min(appliedVolts, 12.0);
                } else if (currentRPM > velocityRPM + tolerance) {
                    appliedVolts = 0.0;
                    torqueCurrentAmps = 0.0;
                }
                // In deadband: maintain previous output (hysteresis)
                break;
        }
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