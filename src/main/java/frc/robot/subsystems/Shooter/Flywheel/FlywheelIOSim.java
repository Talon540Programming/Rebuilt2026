package frc.robot.subsystems.Shooter.Flywheel;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Shooter.ShooterConstants;

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
    private double torqueCurrentAmps = 0.0;
    private double targetVelocityRPM = 0.0;
    private FlywheelState currentState = FlywheelState.COAST;
    private long shotCount = 0;
    
    // Bang-bang state machine
    private Debouncer idleDebouncer;
    
    // Velocity tolerance for "at setpoint" check (RPM)
    private static final double kVelocityTolerance = 50.0;
    
    // Motor model for torque current simulation
    private static final DCMotor kMotorModel = DCMotor.getKrakenX60(2);
    
    public FlywheelIOSim() {
        idleDebouncer = new Debouncer(
            ShooterConstants.flywheelBangBangDebounceSeconds.get(),
            DebounceType.kRising
        );
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
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
        inputs.state = currentState;
        inputs.shotCount = shotCount;
    }
    
   @Override
    public void stop() {
        targetVelocityRPM = 0.0;
        appliedVolts = 0.0;
        torqueCurrentAmps = 0.0;
        currentState = FlywheelState.COAST;
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        // Simulation doesn't need brake mode
    }
    
    @Override
    public void runBangBang(double velocityRPM) {
        targetVelocityRPM = velocityRPM;
        
        // Calculate if we're within tolerance
        double currentRPM = sim.getAngularVelocityRPM();
        double error = Math.abs(currentRPM - velocityRPM);
        boolean inTolerance = error <= ShooterConstants.flywheelTorqueCurrentTolerance.get();
        boolean readyForIdle = idleDebouncer.calculate(inTolerance);
        
        // State machine transitions
        switch (currentState) {
            case COAST:
                if (velocityRPM > 0) {
                    currentState = FlywheelState.STARTUP;
                }
                break;
                
            case STARTUP:
                if (velocityRPM <= 0) {
                    currentState = FlywheelState.COAST;
                } else if (readyForIdle) {
                    currentState = FlywheelState.IDLE;
                }
                break;
                
            case IDLE:
                if (velocityRPM <= 0) {
                    currentState = FlywheelState.COAST;
                } else if (!inTolerance) {
                    currentState = FlywheelState.RECOVERY;
                    shotCount++;
                }
                break;
                
            case RECOVERY:
                if (velocityRPM <= 0) {
                    currentState = FlywheelState.COAST;
                } else if (readyForIdle) {
                    currentState = FlywheelState.IDLE;
                }
                break;
        }
        
        // Apply control based on current state
        switch (currentState) {
            case COAST:
                appliedVolts = 0.0;
                torqueCurrentAmps = 0.0;
                break;
                
            case STARTUP:
            case RECOVERY:
                // Max duty cycle for fastest spinup/recovery
                torqueCurrentAmps = 0.0;
                if (currentRPM < velocityRPM) {
                    appliedVolts = 12.0;
                } else {
                    appliedVolts = 0.0;
                }
                break;
                
            case IDLE:
                // Constant torque current for consistent ball exit velocity
                if (currentRPM < velocityRPM) {
                    torqueCurrentAmps = ShooterConstants.flywheelBangBangTorqueCurrent.get();
                    // Convert torque current to voltage: V = I * R + omega / Kv
                    double omegaRadPerSec = currentRPM * 2.0 * Math.PI / 60.0;
                    appliedVolts = (torqueCurrentAmps * kMotorModel.rOhms) 
                        + (omegaRadPerSec / kMotorModel.KvRadPerSecPerVolt);
                    appliedVolts = Math.min(appliedVolts, 12.0);
                } else {
                    appliedVolts = 0.0;
                    torqueCurrentAmps = 0.0;
                }
                break;
        }
    }
    @Override
    public void simulateShot() {
        double currentRPM = sim.getAngularVelocityRPM();
        double dropRPM = currentRPM;
        double newRPM = currentRPM - dropRPM;
        
        // Convert RPM to rad/s and set the sim state
        double newAngularVelocity = newRPM * 2.0 * Math.PI / 60.0;
        sim.setAngularVelocity(newAngularVelocity);
    }
}