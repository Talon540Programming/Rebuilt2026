package frc.robot.subsystems.Index;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.CANrangeConfiguration;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IndexIOKraken implements IndexIO {
    
    private final TalonFX motor;
    private final CANrange canRange;
    
    // Status signals for motor
    private final StatusSignal<AngularVelocity> velocityRotPerSec;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> tempCelsius;
    private final StatusSignal<Boolean> isDetected;
    private final StatusSignal<Distance> distance;    
    // Control request
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    
    
    public IndexIOKraken() {
        motor = new TalonFX(IndexConstants.kIndexMotorId);
        canRange = new CANrange(IndexConstants.kCANRangeId);
        // Configure CANRange proximity detection threshold
        CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
        canRangeConfig.ProximityParams.ProximityThreshold = IndexConstants.gamePieceDetectionThreshold.get();
        canRange.getConfigurator().apply(canRangeConfig);
        
        // Configure motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Current limits
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Motor direction - may need to change based on testing
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        motor.getConfigurator().apply(config);
        
        // Get status signals
        velocityRotPerSec = motor.getVelocity();
        appliedVolts = motor.getMotorVoltage();
        currentAmps = motor.getStatorCurrent();
        tempCelsius = motor.getDeviceTemp();

        
        // CANRange distance signal
        isDetected = canRange.getIsDetected();
        distance = canRange.getDistance();
        
        // Set update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            velocityRotPerSec,
            appliedVolts,
            currentAmps,
            tempCelsius,
            isDetected,
            distance
        );
        
        // Optimize bus utilization
        motor.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs(IndexIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            velocityRotPerSec,
            appliedVolts,
            currentAmps,
            tempCelsius,
            isDetected,
            distance
        );
        
        inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
        
        // CANRange readings - using built-in proximity detection
        inputs.hasGamePiece = isDetected.getValue();
        inputs.distanceMeters = 0.0; // Distance not needed when using isDetected
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
    public void setBrakeMode(boolean brake) {
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}