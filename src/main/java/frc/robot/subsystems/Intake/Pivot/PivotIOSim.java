package frc.robot.subsystems.Intake.Pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.Intake.IntakeConstants;

public class PivotIOSim implements PivotIO {
    
    // Physical parameters - adjust to match your mechanism
    private static final double kGearRatio = IntakeConstants.kPivotSensorToMechanismRatio;
    private static final double kMOI = 0.5;        // kg*m^2
    private static final double kArmLength = 0.3;  // meters
    
    // Sim hard stops (in radians for the sim, will convert to rotations for output)
    private static final double kMinAngleRad = Math.PI/8;                    // Stowed
    private static final double kMaxAngleRad = Math.toRadians(100);    // Deployed
    
    private final SingleJointedArmSim sim;
    private final DCMotor gearbox = DCMotor.getKrakenX44(1);
    
    private double appliedVolts = 0.0;
    private double positionOffsetRot = 0.0;
    
    // Closed loop state
    private boolean closedLoop = false;
    private double targetRot = 0.0;
    
    // For simulating external disturbances (like collisions)
    private boolean simulateCollision = false;
    
    public PivotIOSim() {
        sim = new SingleJointedArmSim(
            gearbox,
            kGearRatio,
            kMOI,
            kArmLength,
            kMinAngleRad,
            kMaxAngleRad,
            true,  // Simulate gravity
            kMinAngleRad  // Start at stowed position
        );

        positionOffsetRot = kMinAngleRad / (2 * Math.PI);
    }
    
    @Override
    public void updateInputs(PivotIOInputs inputs) {
        // Run closed loop control if enabled
        if (closedLoop) {
            double currentPosRot = getPositionRotations();
            double errorRot = targetRot - currentPosRot;
            
            // Simple P control to simulate Motion Magic
            double kP = 40.0;  // Tune for reasonable sim behavior
            appliedVolts = kP * errorRot;
            appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
        }
        
        // Apply voltage to sim
        sim.setInputVoltage(appliedVolts);
        sim.update(0.02);
        
        // Calculate current draw
        double velocityRadPerSec = sim.getVelocityRadPerSec();
        double currentAmps = calculateCurrentDraw(appliedVolts, velocityRadPerSec);
        
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
        inputs.velocityRotPerSec = velocityRadPerSec / (2 * Math.PI);
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = currentAmps;
        inputs.tempCelsius = 25.0 + (currentAmps * 0.5);  // Fake temp rise with current
    }
    
    private double getPositionRotations() {
        return (sim.getAngleRads() / (2 * Math.PI)) - positionOffsetRot;
    }
    
    private double calculateCurrentDraw(double voltage, double velocityRadPerSec) {
        // I = (V - omega * Kv) / R
        double backEmf = velocityRadPerSec * gearbox.KvRadPerSecPerVolt;
        double current = (voltage - backEmf) / gearbox.rOhms;
        return Math.abs(current);
    }
    
    private boolean isAtHardStop() {
        double angleRad = sim.getAngleRads();
        return angleRad <= kMinAngleRad + 0.01 || angleRad >= kMaxAngleRad - 0.01;
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
        double rawRot = sim.getAngleRads() / (2 * Math.PI);
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
    public void setSimulationAngle(double angleRad) {
        sim.setState(angleRad, 0.0);
    }
    
    /**
     * Check if sim is at the hard stop (useful for verifying homing).
     */
    public boolean isSimAtHardStop() {
        return isAtHardStop();
    }
}