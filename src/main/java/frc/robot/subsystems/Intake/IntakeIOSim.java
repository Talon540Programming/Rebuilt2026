package frc.robot.subsystems.Intake;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.Shooter.ShooterConstants;

/**
 * Simulation implementation of IntakeIO.
 * 
 * This class simulates the intake roller physics for testing and development
 * without physical hardware. Uses MotionMagic-style velocity control with
 * acceleration limiting. Compatible with AdvantageScope visualization.
 */
public class IntakeIOSim implements IntakeIO {
    
    // Roller simulation model
    private final FlywheelSim rollerSim;
    
    // PID controller for closed-loop simulation
    private final PIDController pid;
    
    // State tracking
    private double appliedVolts = 0.0;
    private double targetVelocity = 0.0;
    private double currentSetpoint = 0.0; // Intermediate setpoint for acceleration limiting
    private boolean closedLoopMode = false;
    
    public IntakeIOSim() {

        LinearSystem<N1, N1, N1> rollerPLant =
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(1),
                ShooterConstants.TOP_FLYWHEEL_GEAR_RATIO,
                ShooterConstants.SIM_TOP_MOI
            );

        rollerSim = new FlywheelSim(rollerPLant, DCMotor.getKrakenX44(1), ShooterConstants.TOP_FLYWHEEL_GEAR_RATIO);

        
        // Initialize PID controller for simulation
        pid = new PIDController(
            IntakeConstants.kP,
            IntakeConstants.kI,
            IntakeConstants.kD
        );
    }
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Update simulation physics (20ms loop time)
        if (closedLoopMode) {
            // Simulate MotionMagic acceleration limiting
            double maxAccel = IntakeConstants.ACCELERATION * 0.02; // acceleration * dt
            double velocityError = targetVelocity - currentSetpoint;
            
            if (Math.abs(velocityError) <= maxAccel) {
                currentSetpoint = targetVelocity;
            } else {
                currentSetpoint += Math.signum(velocityError) * maxAccel;
            }
            
            // Closed-loop control: PID + feedforward
            double ff = currentSetpoint * IntakeConstants.kV 
                + Math.signum(currentSetpoint) * IntakeConstants.kS;
            
            double pidOutput = pid.calculate(getVelocityRPS(), currentSetpoint);
            
            appliedVolts = pidOutput + ff;
        }
        
        // Clamp voltage to realistic limits
        appliedVolts = Math.max(-12.0, Math.min(12.0, appliedVolts));
        
        // Update simulation
        rollerSim.setInputVoltage(appliedVolts);
        rollerSim.update(0.02);
        
        // Update inputs
        inputs.velocityRPS = getVelocityRPS();
        inputs.appliedVolts = appliedVolts;
        
        // Simulate current draw
        inputs.supplyCurrentAmps = Math.abs(appliedVolts) * 5.0; // Simplified
        inputs.statorCurrentAmps = rollerSim.getCurrentDrawAmps();
        
        // Simulate temperature
        inputs.tempCelsius = 25.0 + inputs.statorCurrentAmps * 0.5;
        
        inputs.targetVelocityRPS = targetVelocity;
    }
    
    @Override
    public void setVelocity(double velocityRPS) {
        closedLoopMode = true;
        targetVelocity = velocityRPS;
    }
    
    @Override
    public void stop() {
        closedLoopMode = false;
        appliedVolts = 0.0;
        targetVelocity = 0.0;
        currentSetpoint = 0.0;
        pid.reset();
    }
    
    @Override
    public void setVoltage(double volts) {
        closedLoopMode = false;
        appliedVolts = volts;
        targetVelocity = 0.0;
        currentSetpoint = 0.0;
        pid.reset();
    }
    
    /**
     * Gets the simulated roller velocity in rotations per second.
     * Converts from rad/s to rps.
     */
    private double getVelocityRPS() {
        return rollerSim.getAngularVelocityRPM() / 60.0;
    }
}