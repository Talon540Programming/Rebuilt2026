package frc.robot.subsystems.Shooter.Hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;

public class HoodIOKraken implements HoodIO {
    
    private final TalonFX motor;
    
    // Status signals
    private final StatusSignal<Angle> positionRotations;
    private final StatusSignal<AngularVelocity> velocityRotPerSec;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> tempCelsius;
    
    // Control requests
    private final PositionVoltage positionControl = new PositionVoltage(0);
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    
    private double targetPositionRadians = 0.0;
    
    // Gear ratio from motor to hood output (rack and pinion)
    // Adjust based on actual mechanism - this converts motor rotations to hood radians
    private static final double kGearRatio = 50.0; // Example: 50 motor rotations per radian of hood
    
    // Position tolerance for "at setpoint" check (radians)
    private static final double kPositionTolerance = Math.toRadians(2.0);
    
    public HoodIOKraken() {
        motor = new TalonFX(ShooterConstants.kHoodMotorId);
        
        // Configure motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Current limits - X44
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Motor direction - may need to change based on testing
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Position PID - slot 0
        config.Slot0.kP = ShooterConstants.kHoodkP;
        config.Slot0.kI = ShooterConstants.kHoodkI;
        config.Slot0.kD = ShooterConstants.kHoodkD;
        
        // Software limits to prevent over-travel
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 
            ShooterConstants.kHoodMaxAngle * kGearRatio / (2 * Math.PI); // Convert to rotations
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 
            ShooterConstants.kHoodMinAngle * kGearRatio / (2 * Math.PI);
        
        motor.getConfigurator().apply(config);
        
        // Get status signals
        positionRotations = motor.getPosition();
        velocityRotPerSec = motor.getVelocity();
        appliedVolts = motor.getMotorVoltage();
        currentAmps = motor.getStatorCurrent();
        tempCelsius = motor.getDeviceTemp();
        
        // Set update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            positionRotations,
            velocityRotPerSec,
            appliedVolts,
            currentAmps,
            tempCelsius
        );
        
        // Optimize bus utilization
        motor.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs(HoodIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            positionRotations,
            velocityRotPerSec,
            appliedVolts,
            currentAmps,
            tempCelsius
        );
        
        inputs.positionRotations = positionRotations.getValueAsDouble();
        // Convert motor rotations to hood radians
        inputs.positionRadians = (inputs.positionRotations / kGearRatio) * (2 * Math.PI);
        inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
        inputs.targetPositionRadians = targetPositionRadians;
        inputs.atSetpoint = Math.abs(inputs.positionRadians - targetPositionRadians) < kPositionTolerance;
    }
    
    @Override
    public void setPosition(double positionRadians) {
        // Clamp to valid range
        positionRadians = Math.max(ShooterConstants.kHoodMinAngle, 
                         Math.min(ShooterConstants.kHoodMaxAngle, positionRadians));
        targetPositionRadians = positionRadians;
        
        // Convert hood radians to motor rotations
        double motorRotations = (positionRadians / (2 * Math.PI)) * kGearRatio;
        motor.setControl(positionControl.withPosition(motorRotations));
    }
    
    @Override
    public void setDutyCycle(double dutyCycle) {
        motor.setControl(dutyCycleControl.withOutput(dutyCycle));
    }
    
    @Override
    public void stop() {
        motor.setControl(dutyCycleControl.withOutput(0));
    }
    
    @Override
    public void zeroEncoder() {
        // Zero the encoder - assumes hood is at minimum position when called
        motor.setPosition((ShooterConstants.kHoodMinAngle / (2 * Math.PI)) * kGearRatio);
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}