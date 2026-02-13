package frc.robot.subsystems.Shooter.Hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO need to tune for real bot
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
        
        // Software limits (in motor rotations)
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterConstants.hoodMaxPositionRot.get() - 0.01;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShooterConstants.hoodMinPositionRot.get() + 0.01;
        
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
        // Convert motor position (rotations) to hood angle (radians)
        // Reverse linear interpolation
        double minAngle = ShootingConstants.hoodMinAngle;
        double maxAngle = ShootingConstants.hoodMaxAngle;
        double minPos = ShooterConstants.hoodMinPositionRot.get();
        double maxPos = ShooterConstants.hoodMaxPositionRot.get();
        
        double t = (inputs.positionRotations - minPos) / (maxPos - minPos);
        inputs.positionRadians = minAngle + t * (maxAngle - minAngle);
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
        
        // Map hood angle (radians) to motor position (rotations)
        // Linear interpolation from min angle -> min position, max angle -> max position
        double minAngle = ShootingConstants.hoodMinAngle;
        double maxAngle = ShootingConstants.hoodMaxAngle;
        double minPos = ShooterConstants.hoodMinPositionRot.get();
        double maxPos = ShooterConstants.hoodMaxPositionRot.get();
        
        double t = (positionRadians - minAngle) / (maxAngle - minAngle);
        double motorRotations = minPos + t * (maxPos - minPos);
        
        motor.setControl(positionControl.withPosition(motorRotations));
    }
    
    @Override
    public void zeroEncoder() {
        // Zero the encoder - assumes hood is at minimum position when called
        motor.setPosition(ShooterConstants.hoodMinPositionRot.get());
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setEncoderPosition(double positionRotations) {
        motor.setPosition(positionRotations);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        motor.set(dutyCycle);
    }

    @Override
    public void setSoftLimitsEnabled(boolean enabled) {
        var config = new SoftwareLimitSwitchConfigs();
        config.ForwardSoftLimitEnable = enabled;
        config.ReverseSoftLimitEnable = enabled;
        if (enabled) {
            double maxAngleRotations = ShootingConstants.hoodMaxAngle / (2 * Math.PI);
            double minAngleRotations = ShootingConstants.hoodMinAngle / (2 * Math.PI);
            config.ForwardSoftLimitThreshold = maxAngleRotations + 0.01;
            config.ReverseSoftLimitThreshold = minAngleRotations - 0.01;
        }
        motor.getConfigurator().apply(config);
    }
}