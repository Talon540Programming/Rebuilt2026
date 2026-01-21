package frc.robot.subsystems.Shooter.Hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVoltage;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShootingConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Shooter.ShooterConstants;

public class HoodIOKraken implements HoodIO {
    
    private final TalonFX motor;
    
    // Status signals
    private final StatusSignal<Angle> positionRotations;
    private final StatusSignal<AngularVelocity> velocityRotPerSec;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> tempCelsius;
    
    // Control requests
    private final MotionMagicVoltage positionControl = new MotionMagicVoltage(0);
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    
    private double targetPositionRadians = 0.0;
    
    
    public HoodIOKraken() {
        motor = new TalonFX(ShooterConstants.hoodMotorId, TunerConstants.kCANBus);
        
        // Configure motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Current limits
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.hoodStatorCurrentLimit.get();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.hoodSupplyCurrentLimit.get();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Motor direction
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Gear ratio: converts rotor rotations to mechanism rotations
        config.Feedback.SensorToMechanismRatio = ShooterConstants.hoodSensorToMechanismRatio.get();
        
        // Slot 0 gains for Motion Magic Position
        config.Slot0.kP = ShooterConstants.hoodkP.get();
        config.Slot0.kI = ShooterConstants.hoodkI.get();
        config.Slot0.kD = ShooterConstants.hoodkD.get();
        config.Slot0.kS = ShooterConstants.hoodkS.get();
        config.Slot0.kV = ShooterConstants.hoodkV.get();
        config.Slot0.kA = ShooterConstants.hoodkA.get();
        // No kG for rack and pinion (no gravity compensation needed)
        
        // Motion Magic Position settings
        config.MotionMagic.MotionMagicCruiseVelocity = ShooterConstants.hoodMMCruiseVelRotPerSec.get();
        config.MotionMagic.MotionMagicAcceleration = ShooterConstants.hoodMMAccelRotPerSec2.get();
        config.MotionMagic.MotionMagicJerk = ShooterConstants.hoodMMJerkRotPerSec3.get();
        
        // Software limits (in mechanism rotations)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShootingConstants.hoodMaxAngle + 0.01;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShootingConstants.hoodMinAngle - 0.01;
        
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
        // Convert mechanism rotations to radians
        inputs.positionRadians = inputs.positionRotations * (2 * Math.PI);
        inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
        inputs.targetPositionRadians = targetPositionRadians;
        
        double targetPosRot = targetPositionRadians / (2 * Math.PI);
        inputs.atSetpoint = Math.abs(inputs.positionRotations - targetPosRot) < ShooterConstants.hoodPosToleranceRot.get()
            && Math.abs(inputs.velocityRotPerSec) < ShooterConstants.hoodVelToleranceRotPerSec.get();
    }
    
    @Override
    public void setPosition(double positionRadians) {
        targetPositionRadians = positionRadians;
        
        // Convert radians to mechanism rotations
        double mechanismRotations = positionRadians / (2 * Math.PI);
        motor.setControl(positionControl.withPosition(mechanismRotations));
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
        motor.setPosition(ShootingConstants.hoodMinAngle);
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setEncoderPosition(double positionRotations) {
        motor.setPosition(positionRotations);
}
}