package frc.robot.subsystems.Intake.Extension;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.subsystems.Intake.IntakeConstants;

public class ExtensionIOSim implements ExtensionIO {

    // Constants for simulation
    private static final double kGearRatio = IntakeConstants.extensionSensorToMechanismRatio.get();
    private static final double kCarriageMassKg = Units.lbsToKilograms(5);  // ~5 lbs - TODO: update when known
    private static final double kDrumRadiusMeters = 0.02;  // Pinion radius - TODO: update when known

    // Sim hard stops (in meters)
    private static final double kMinHeightMeters = 0.0;   // Fully retracted
    private static final double kMaxHeightMeters = 0.5;   // Fully extended - TODO: update when known
    
    private final ElevatorSim sim;
    private final DCMotor gearbox = DCMotor.getKrakenX44(1);
    
    private double appliedVolts = 0.0;
    private double positionOffsetRot = 0.0;
    
    // Closed loop state
    private boolean closedLoop = false;
    private double targetRot = 0.0;
    
    // For simulating external disturbances (like collisions)
    private boolean simulateCollision = false;
    
    public ExtensionIOSim() {
        sim = new ElevatorSim(
            gearbox,
            kGearRatio,
            kCarriageMassKg,
            kDrumRadiusMeters,
            kMinHeightMeters,
            kMaxHeightMeters,
            false,  // Don't simulate gravity (horizontal/self-supporting)
            kMinHeightMeters  // Starting position
        );

        positionOffsetRot = 0.0;
    }

    
    @Override
    public void updateInputs(PivotIOInputs inputs) {
        // Run closed loop control if enabled
        if (closedLoop) {
            double currentPosRot = getPositionRotations();
            double errorRot = targetRot - currentPosRot;
            
            // Simple P control to simulate Motion Magic
            double kP = IntakeConstants.extensionkP.get();  // Tune for reasonable sim behavior
            appliedVolts = kP * errorRot;
            appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
        }
        
        // Apply voltage to sim
        sim.setInputVoltage(appliedVolts);
        sim.update(0.02);
        
        // Calculate current draw
        double velocityMetersPerSec = sim.getVelocityMetersPerSecond();
        double currentAmps = calculateCurrentDraw(appliedVolts, velocityMetersPerSec);
        
        // If at hard stop and trying to push further, simulate stall current
        if (isAtHardStop() && Math.abs(appliedVolts) > 0.5) {
            currentAmps = Math.abs(appliedVolts) / gearbox.rOhms * 0.8;  // Approximate stall current
        }
        
        // If simulating collision, add extra current
        if (simulateCollision) {
            currentAmps += 15.0;  // Spike current during collision
        }
        
        // Fill inputs
        inputs.positionRotations = getPositionRotations();
        double metersPerRotation = (2 * Math.PI * kDrumRadiusMeters) / kGearRatio;
        inputs.velocityRotPerSec = sim.getVelocityMetersPerSecond() / metersPerRotation;
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = currentAmps;
        inputs.tempCelsius = 25.0 + (currentAmps * 0.5);  // Fake temp rise with current
    }
    
    // Replace with:
    private double getPositionRotations() {
        // Convert linear position (meters) to motor rotations
        double metersPerRotation = (2 * Math.PI * kDrumRadiusMeters) / kGearRatio;
        return (sim.getPositionMeters() / metersPerRotation) - positionOffsetRot;
    }

    
    private double calculateCurrentDraw(double voltage, double velocityMetersPerSec) {
        // Convert linear velocity to rotational for back-EMF calculation
        double velocityRadPerSec = velocityMetersPerSec / kDrumRadiusMeters;
        double backEmf = velocityRadPerSec / gearbox.KvRadPerSecPerVolt;
        double current = (voltage - backEmf) / gearbox.rOhms;
        return Math.abs(current);
    }
    
    private boolean isAtHardStop() {
        double position = sim.getPositionMeters();
        return position <= kMinHeightMeters + 0.001 || position >= kMaxHeightMeters - 0.001;
    }

    
    @Override
    public void setDutyCycle(double dutyCycle) {
        closedLoop = false;
        appliedVolts = dutyCycle * 12.0;
    }
    
    @Override
    public void stop() {
        closedLoop = false;
        appliedVolts = 0.0;
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        // Could simulate brake mode by adding damping, but not critical
    }
    
    @Override
    public void runMotionMagicPosition(double positionRotations, double feedForwardVolts) {
        closedLoop = true;
        targetRot = positionRotations;
    }
    
    @Override
    public void setPosition(double positionRotations) {
        double metersPerRotation = (2 * Math.PI * kDrumRadiusMeters) * kGearRatio;
        double rawRot = sim.getPositionMeters() / metersPerRotation;
        positionOffsetRot = rawRot - positionRotations;
    }
    
    // ==================== SIMULATION TEST METHODS ====================
    
    /**
     * Simulate a collision pushing the intake back.
     * Call this from a test or dashboard button.
     */
    public void simulateCollision(boolean colliding) {
        this.simulateCollision = colliding;
    }
    
    /**
     * Manually set the arm position (for testing scenarios).
     * @param angleRad Angle in radians
     */
    public void setSimulationPosition(double positionMeters) {
        sim.setState(positionMeters, 0.0);
    }
    
    /**
     * Check if sim is at the hard stop (useful for verifying homing).
     */
    public boolean isSimAtHardStop() {
        return isAtHardStop();
    }
}