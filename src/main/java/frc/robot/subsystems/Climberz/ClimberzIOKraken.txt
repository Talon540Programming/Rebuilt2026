package frc.robot.subsystems.Climberz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

public class ClimberzIOKraken implements ClimberzIO {
    
    private final TalonFX motor;
    
    // Status signals
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> current;
    private final StatusSignal<Temperature> temp;
    
    // Control request
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);

    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0);
    
    public ClimberzIOKraken() {
        motor = new TalonFX(ClimberzConstants.motorId, TunerConstants.kCANBus);
        
        // Configure motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Gear ratio
        config.Feedback.SensorToMechanismRatio = ClimberzConstants.sensorToMechanismRatio;

        // Slot0 PID
        config.Slot0.kP = ClimberzConstants.kP.get();
        config.Slot0.kI = ClimberzConstants.kI.get();
        config.Slot0.kD = ClimberzConstants.kD.get();
        config.Slot0.kS = ClimberzConstants.kS.get();
        config.Slot0.kV = ClimberzConstants.kV.get();
        config.Slot0.kG = ClimberzConstants.kG.get();
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // Motion Magic constraints
        config.MotionMagic.MotionMagicCruiseVelocity = ClimberzConstants.mmCruiseVelRotPerSec.get();
        config.MotionMagic.MotionMagicAcceleration = ClimberzConstants.mmAccelRotPerSec2.get();
        config.MotionMagic.MotionMagicJerk = ClimberzConstants.mmJerkRotPerSec3.get();
        
        motor.getConfigurator().apply(config);

        
        // Get status signals
        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVolts = motor.getMotorVoltage();
        current = motor.getStatorCurrent();
        temp = motor.getDeviceTemp();
        
        // Set update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            position, velocity, appliedVolts, current, temp
        );
        
        // Optimize bus utilization
        motor.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs(ClimberzIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, current, temp
        );
        
        inputs.positionRotations = position.getValueAsDouble();
        inputs.velocityRotPerSec = velocity.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }
    
    @Override
    public void setDutyCycle(double dutyCycle) {
        motor.setControl(dutyCycleControl.withOutput(dutyCycle));
    }
    
    @Override
    public void stop() {
        motor.stopMotor();
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motor.setNeutralMode(mode);
    }

    @Override
    public void runMotionMagicPosition(double positionRotations) {
        motor.setControl(mmRequest.withPosition(positionRotations));
    }

    @Override
    public void setPosition(double positionRotations) {
        motor.setPosition(positionRotations);
    }
}