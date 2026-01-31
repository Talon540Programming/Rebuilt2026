package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * Hardware implementation of IntakeIO using TalonFX motor controller.
 * 
 * This class interfaces with the physical X44 motor via CTRE Phoenix 6 API.
 * Uses MotionMagic velocity control for smooth acceleration to target speeds.
 */
public class IntakeIOTalonFX implements IntakeIO {
    
    private final TalonFX motor;
    
    // Control requests (reusable objects for performance)
    private final MotionMagicVelocityVoltage velocityRequest;
    private final VoltageOut voltageRequest;
    
    // Status signals for efficient data retrieval
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> temp;
    
    private double targetVelocity = 0.0;
    
    public IntakeIOTalonFX() {
        // Initialize motor
        motor = new TalonFX(IntakeConstants.MOTOR_ID, IntakeConstants.CAN_BUS_NAME);
        
        // Configure motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Configure MotionMagic parameters
        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = IntakeConstants.ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = IntakeConstants.JERK;
        
        // Configure velocity PID
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = IntakeConstants.kP;
        slot0.kI = IntakeConstants.kI;
        slot0.kD = IntakeConstants.kD;
        slot0.kV = IntakeConstants.kV;
        slot0.kS = IntakeConstants.kS;
        slot0.kA = IntakeConstants.kA;
        
        // Configure current limits
        config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = IntakeConstants.SUPPLY_CURRENT_LIMIT_ENABLE;
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = IntakeConstants.STATOR_CURRENT_LIMIT_ENABLE;
        
        // Configure motor output settings
        config.MotorOutput.NeutralMode = IntakeConstants.USE_BRAKE_MODE 
            ? NeutralModeValue.Brake 
            : NeutralModeValue.Coast;
        config.MotorOutput.Inverted = IntakeConstants.INVERT_MOTOR 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        
        // Configure gear ratio for proper velocity reporting
        config.Feedback.SensorToMechanismRatio = IntakeConstants.GEAR_RATIO;
        
        // Apply configuration
        motor.getConfigurator().apply(config);
        
        // Initialize control requests
        velocityRequest = new MotionMagicVelocityVoltage(0).withSlot(IntakeConstants.PID_SLOT);
        voltageRequest = new VoltageOut(0);
        
        // Initialize status signals
        velocity = motor.getVelocity();
        appliedVolts = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        statorCurrent = motor.getStatorCurrent();
        temp = motor.getDeviceTemp();
        
        // Optimize bus utilization by setting update frequencies
        velocity.setUpdateFrequency(100);
        appliedVolts.setUpdateFrequency(50);
        supplyCurrent.setUpdateFrequency(50);
        statorCurrent.setUpdateFrequency(50);
        temp.setUpdateFrequency(4);
    }
    
    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        // Refresh all status signals
        velocity.refresh();
        appliedVolts.refresh();
        supplyCurrent.refresh();
        statorCurrent.refresh();
        temp.refresh();
        
        // Update inputs with current values
        inputs.velocityRPS = velocity.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
        inputs.targetVelocityRPS = targetVelocity;
    }
    
    @Override
    public void setVelocity(double velocityRPS) {
        targetVelocity = velocityRPS;
        motor.setControl(velocityRequest.withVelocity(velocityRPS));
    }
    
    @Override
    public void stop() {
        targetVelocity = 0.0;
        motor.stopMotor();
    }
    
    @Override
    public void setVoltage(double volts) {
        targetVelocity = 0.0;
        motor.setControl(voltageRequest.withOutput(volts));
    }
}