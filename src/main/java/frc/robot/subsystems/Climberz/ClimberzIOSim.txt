package frc.robot.subsystems.Climberz;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberzIOSim implements ClimberzIO {

    // Constants for simulation - AndyMark Climber in a Box with Kraken X60
    private static final double kGearRatio = 16.0;  // Your gear ratio
    private static final double kCarriageMassKg = Units.lbsToKilograms(5);  // Light mass for arm movement simulation
    private static final double kDrumRadiusMeters = 0.0127;  // ~0.5 inch spool radius

    // Sim hard stops (in meters)
    private static final double kMinHeightMeters = 0.0;   // Fully retracted
    private static final double kMaxHeightMeters = 0.6;   // Fully extended (~24 inches per stage)
    
    // When extending, we're just moving the arm (not lifting robot), so use lighter mass equivalent
    private static final double kSpringExtendVolts = 6.0;  // Simulated spring force as voltage
    
    private final ElevatorSim sim;
    private final DCMotor gearbox = DCMotor.getKrakenX60(1);
    
    private double appliedVolts = 0.0;
    private boolean brakeMode = true;
    public ClimberzIOSim() {
        sim = new ElevatorSim(
            gearbox,
            kGearRatio,
            kCarriageMassKg,
            kDrumRadiusMeters,
            kMinHeightMeters,
            kMaxHeightMeters,
            false,  // Don't simulate gravity (springs counteract it when extended)
            kMaxHeightMeters  // Starting position (extended - springs push it out)
        );
    }

    @Override
    public void updateInputs(ClimberzIOInputs inputs) {
        // In coast mode with no voltage, let spring extend the arm
        // Note: Positive voltage = extend (sim convention), but we want:
        //   - Positive duty cycle from user = retract (toward 0)
        //   - Coast mode = springs extend (toward max)
        // So we invert the applied volts for the sim
        double effectiveVolts = -appliedVolts;  // Invert so positive duty cycle retracts
        
        if (!brakeMode && Math.abs(appliedVolts) < 0.1) {
            // Simulate spring pushing arm out (positive voltage in sim = extend)
            effectiveVolts = kSpringExtendVolts;
        }
        
        sim.setInputVoltage(effectiveVolts);
        sim.update(0.02);
        
        // Convert linear position to rotations
        double metersPerRotation = (2 * Math.PI * kDrumRadiusMeters) / kGearRatio;
        
        inputs.positionRotations = sim.getPositionMeters() / metersPerRotation;
        inputs.velocityRotPerSec = sim.getVelocityMetersPerSecond() / metersPerRotation;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
        inputs.tempCelsius = 25.0;
    }
    
    @Override
    public void setDutyCycle(double dutyCycle) {
        appliedVolts = dutyCycle * 12.0;
    }
    
    @Override
    public void stop() {
        appliedVolts = 0.0;
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        this.brakeMode = brake;
    }

    @Override
    public void runMotionMagicPosition(double positionRotations) {
        // Simple P control for simulation
        double metersPerRotation = (2 * Math.PI * kDrumRadiusMeters) / kGearRatio;
        double currentPos = sim.getPositionMeters() / metersPerRotation;
        double error = positionRotations - currentPos;
        appliedVolts = error * 2.0;  // Simple proportional control
        appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
    }

    @Override
    public void setPosition(double positionRotations) {
        // Reset sim position
        double metersPerRotation = (2 * Math.PI * kDrumRadiusMeters) / kGearRatio;
        sim.setState(positionRotations * metersPerRotation, 0.0);
    }
}