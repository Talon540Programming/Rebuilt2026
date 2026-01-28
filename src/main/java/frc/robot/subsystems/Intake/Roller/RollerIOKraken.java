package frc.robot.subsystems.Intake.Roller;

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
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake.IntakeConstants;


public class RollerIOKraken implements RollerIO {
    
    private final TalonFX roller;
    
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> current;
    private final StatusSignal<Temperature> temp;
    
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    
    public RollerIOKraken() {
        roller = new TalonFX(IntakeConstants.rollerMotorId, TunerConstants.kCANBus);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Current limits for X44
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Motor direction - adjust based on your mechanism
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        roller.getConfigurator().apply(config);
        
        // Get status signals
        velocity = roller.getVelocity();
        appliedVolts = roller.getMotorVoltage();
        current = roller.getStatorCurrent();
        temp = roller.getDeviceTemp();
        
        // Set update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(50, velocity, appliedVolts, current, temp);
        roller.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs(RollerIOInputs inputs) {
        BaseStatusSignal.refreshAll(velocity, appliedVolts, current, temp);
        
        inputs.velocityRotPerSec = velocity.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();

         inputs.hasGamePiece = Math.abs(inputs.velocityRotPerSec) > 0.1  // motor is running
        && inputs.currentAmps > IntakeConstants.gamePieceCurrentThreshold.get();
    }
    
    @Override
    public void setDutyCycle(double dutyCycle) {
        roller.setControl(dutyCycleControl.withOutput(dutyCycle));
    }
    
    @Override
    public void stop() {
        roller.stopMotor();
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        roller.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }
}