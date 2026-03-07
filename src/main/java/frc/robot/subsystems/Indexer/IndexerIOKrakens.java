package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;


public class IndexerIOKrakens implements IndexerIO {
    
    private final TalonFX motor; 

    // Status Signals 
    private final StatusSignal<AngularVelocity> velocityRotPerSec;
    private final StatusSignal<Voltage> appliedVoltage;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> temp;

    // Control Request 
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    
    public IndexerIOKrakens() {
        
        motor = new TalonFX(15);

        // Configuring the motor
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Current limits 
        config.CurrentLimits.StatorCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        //Motor Direction
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor.getConfigurator().apply(config);

        // Getting Status Signals
        velocityRotPerSec = motor.getVelocity();
        appliedVoltage = motor.getMotorVoltage();
        currentAmps = motor.getStatorCurrent();
        temp = motor.getDeviceTemp();


        // Update Frequencies 
        BaseStatusSignal.setUpdateFrequencyForAll(
            50, 
            velocityRotPerSec,
            appliedVoltage,
            currentAmps,
            temp
        );

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            velocityRotPerSec,
            appliedVoltage,
            currentAmps,
            temp
        );

        inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
        inputs.appliedVoltage = appliedVoltage.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.temp = temp.getValueAsDouble();
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
    public void breakMode(boolean brake) {
        motor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

}