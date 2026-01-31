package frc.robot.subsystems.Hood;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * Hardware implementation of HoodIO using TalonFX motor controller.
 * 
 * This class interfaces with the physical X44 Kraken motor via CTRE Phoenix 6 API.
 * Uses MotionMagic position control for smooth movement of the rack and pinion
 * hood angle adjustment between pi/8 and pi/2 radians.
 */
public class HoodIOTalonFX implements HoodIO {
    
    private final TalonFX motor;
    
    // Control requests (reusable objects for performance)
    private final MotionMagicVoltage positionRequest;
    private final VoltageOut voltageRequest;
    
    // Status signals for efficient data retrieval
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> supplyCurrent;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Temperature> temp;
    
    private double targetPosition = 0.0;
    
    public HoodIOTalonFX() {
        // Initialize motor with 2026 constructor
        CANBus canBus = new CANBus(HoodConstants.CAN_BUS_NAME);
        motor = new TalonFX(HoodConstants.MOTOR_ID, canBus);
        
        // Configure motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Configure MotionMagic parameters
        MotionMagicConfigs motionMagicConfigs = config.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = HoodConstants.CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = HoodConstants.ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = HoodConstants.JERK;
        
        // Configure position PID with gravity compensation
        Slot0Configs slot0 = config.Slot0;
        slot0.kP = HoodConstants.kP;
        slot0.kI = HoodConstants.kI;
        slot0.kD = HoodConstants.kD;
        slot0.kV = HoodConstants.kV;
        slot0.kS = HoodConstants.kS;
        slot0.kA = HoodConstants.kA;
        slot0.kG = HoodConstants.kG;
        slot0.GravityType = GravityTypeValue.Arm_Cosine; // Gravity compensation for angled mechanism
        
        // Configure software limits to prevent over-rotation
        SoftwareLimitSwitchConfigs limitConfigs = config.SoftwareLimitSwitch;
        limitConfigs.ForwardSoftLimitEnable = true;
        limitConfigs.ForwardSoftLimitThreshold = HoodConstants.MAX_ANGLE_ROTATIONS;
        limitConfigs.ReverseSoftLimitEnable = true;
        limitConfigs.ReverseSoftLimitThreshold = HoodConstants.MIN_ANGLE_ROTATIONS;
        
        // Configure current limits
        config.CurrentLimits.SupplyCurrentLimit = HoodConstants.SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = HoodConstants.SUPPLY_CURRENT_LIMIT_ENABLE;
        config.CurrentLimits.StatorCurrentLimit = HoodConstants.STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = HoodConstants.STATOR_CURRENT_LIMIT_ENABLE;
        
        // Configure motor output settings
        config.MotorOutput.NeutralMode = HoodConstants.USE_BRAKE_MODE 
            ? NeutralModeValue.Brake 
            : NeutralModeValue.Coast;
        config.MotorOutput.Inverted = HoodConstants.INVERT_MOTOR 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        
        // Configure gear ratio for proper position reporting
        config.Feedback.SensorToMechanismRatio = HoodConstants.GEAR_RATIO;
        
        // Apply configuration
        motor.getConfigurator().apply(config);
        
        // Initialize control requests
        positionRequest = new MotionMagicVoltage(0).withSlot(HoodConstants.PID_SLOT);
        voltageRequest = new VoltageOut(0);
        
        // Initialize status signals
        position = motor.getPosition();
        velocity = motor.getVelocity();
        appliedVolts = motor.getMotorVoltage();
        supplyCurrent = motor.getSupplyCurrent();
        statorCurrent = motor.getStatorCurrent();
        temp = motor.getDeviceTemp();
        
        // Optimize bus utilization by setting update frequencies
        position.setUpdateFrequency(100);
        velocity.setUpdateFrequency(100);
        appliedVolts.setUpdateFrequency(50);
        supplyCurrent.setUpdateFrequency(50);
        statorCurrent.setUpdateFrequency(50);
        temp.setUpdateFrequency(4);
    }
    
    @Override
    public void updateInputs(HoodIOInputs inputs) {
        // Refresh all status signals
        position.refresh();
        velocity.refresh();
        appliedVolts.refresh();
        supplyCurrent.refresh();
        statorCurrent.refresh();
        temp.refresh();
        
        // Update inputs with current values
        inputs.positionRotations = position.getValueAsDouble();
        inputs.velocityRPS = velocity.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
        inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
        inputs.targetPositionRotations = targetPosition;
    }
    
    @Override
    public void setPosition(double positionRotations) {
        // Clamp position to software limits
        targetPosition = Math.max(HoodConstants.MIN_ANGLE_ROTATIONS,
                         Math.min(HoodConstants.MAX_ANGLE_ROTATIONS, positionRotations)); //use clamp 
        
        motor.setControl(positionRequest.withPosition(targetPosition));
    }
    
    @Override
    public void zeroEncoder() {
        motor.setPosition(0.0);
        targetPosition = 0.0;
    }
    
    @Override
    public void stop() {
        targetPosition = position.getValueAsDouble(); // Hold current position
        motor.stopMotor();
    }
    
    @Override
    public void setVoltage(double volts) {
        motor.setControl(voltageRequest.withOutput(volts));
    }
}