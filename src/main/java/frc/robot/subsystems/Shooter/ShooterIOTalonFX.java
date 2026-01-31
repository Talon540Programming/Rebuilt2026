package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * Hardware implementation of ShooterIO using TalonFX motor controllers.
 * 
 * This class interfaces with the physical Kraken X60 motors via CTRE Phoenix 6 API.
 * Uses a leader-follower configuration where:
 * - Bottom motor (leader): runs velocity control
 * - Top motor (follower): mechanically follows the leader motor
 * 
 * The top wheel is 3x smaller than the bottom wheel, so it naturally spins 3x faster
 * when following the same motor output due to the mechanical gear ratio difference.
 */
public class ShooterIOTalonFX implements ShooterIO {
    
    private final TalonFX bottomMotor;
    private final TalonFX topMotor;
    
    // Control requests (reusable objects for performance)
    private final VelocityVoltage velocityRequest;
    private final VoltageOut voltageRequest;
    
    // Status signals for efficient data retrieval
    private final StatusSignal<AngularVelocity> bottomVelocity;
    private final StatusSignal<AngularVelocity> topVelocity;
    private final StatusSignal<Voltage> bottomAppliedVolts;
    private final StatusSignal<Voltage> topAppliedVolts;
    private final StatusSignal<Current> bottomSupplyCurrent;
    private final StatusSignal<Current> topSupplyCurrent;
    private final StatusSignal<Current> bottomStatorCurrent;
    private final StatusSignal<Current> topStatorCurrent;
    private final StatusSignal<Temperature> bottomTemp;
    private final StatusSignal<Temperature> topTemp;
    
    private double targetBottomVelocity = 0.0;
    private double targetTopVelocity = 0.0;
    
    public ShooterIOTalonFX() {
        // Initialize motors
        bottomMotor = new TalonFX(ShooterConstants.LEADER_MOTOR_ID, ShooterConstants.CAN_BUS_NAME);
        topMotor = new TalonFX(ShooterConstants.FOLLOWER_MOTOR_ID, ShooterConstants.CAN_BUS_NAME);
        
        // Configure leader motor (bottom flywheel)
        TalonFXConfiguration bottomConfig = new TalonFXConfiguration();
        
        // Configure velocity PID for bottom motor
        Slot0Configs bottomSlot0 = bottomConfig.Slot0;
        bottomSlot0.kP = ShooterConstants.kP;
        bottomSlot0.kI = ShooterConstants.kI;
        bottomSlot0.kD = ShooterConstants.kD;
        bottomSlot0.kV = ShooterConstants.kV;
        bottomSlot0.kS = ShooterConstants.kS;
        bottomSlot0.kA = ShooterConstants.kA;
        
        // Configure current limits for bottom motor
        bottomConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT;
        bottomConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.SUPPLY_CURRENT_LIMIT_ENABLE;
        bottomConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.STATOR_CURRENT_LIMIT;
        bottomConfig.CurrentLimits.StatorCurrentLimitEnable = ShooterConstants.STATOR_CURRENT_LIMIT_ENABLE;
        
        // Configure motor output settings for bottom motor
        bottomConfig.MotorOutput.NeutralMode = ShooterConstants.USE_BRAKE_MODE 
            ? NeutralModeValue.Brake 
            : NeutralModeValue.Coast;
        bottomConfig.MotorOutput.Inverted = ShooterConstants.INVERT_BOTTOM_MOTOR 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        
        // Configure gear ratio for proper velocity reporting
        bottomConfig.Feedback.SensorToMechanismRatio = ShooterConstants.BOTTOM_FLYWHEEL_GEAR_RATIO;
        
        // Apply configuration to bottom motor
        bottomMotor.getConfigurator().apply(bottomConfig);
        
        // Configure follower motor (top flywheel) with 3:1 gear ratio
        TalonFXConfiguration topConfig = new TalonFXConfiguration();
        
        // Configure current limits for top motor
        topConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SUPPLY_CURRENT_LIMIT;
        topConfig.CurrentLimits.SupplyCurrentLimitEnable = ShooterConstants.SUPPLY_CURRENT_LIMIT_ENABLE;
        topConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.STATOR_CURRENT_LIMIT;
        topConfig.CurrentLimits.StatorCurrentLimitEnable = ShooterConstants.STATOR_CURRENT_LIMIT_ENABLE;
        
        // Configure motor output settings for top motor
        topConfig.MotorOutput.NeutralMode = ShooterConstants.USE_BRAKE_MODE 
            ? NeutralModeValue.Brake 
            : NeutralModeValue.Coast;
        topConfig.MotorOutput.Inverted = ShooterConstants.INVERT_TOP_MOTOR 
            ? InvertedValue.Clockwise_Positive 
            : InvertedValue.CounterClockwise_Positive;
        
        // Configure gear ratio for 3x speed (smaller wheel)
        topConfig.Feedback.SensorToMechanismRatio = ShooterConstants.TOP_FLYWHEEL_GEAR_RATIO;
        
        // Apply configuration to top motor
        topMotor.getConfigurator().apply(topConfig);
        
        // CRITICAL FIX: Top motor must follow the LEADER (bottom) motor
        // The first parameter is the CAN ID of the motor to follow
        // false = same direction as leader (use true for opposite direction)
        topMotor.setControl(new Follower(ShooterConstants.LEADER_MOTOR_ID, MotorAlignmentValue.Aligned));

        // Initialize control requests
        velocityRequest = new VelocityVoltage(0).withSlot(ShooterConstants.VELOCITY_SLOT);
        voltageRequest = new VoltageOut(0);
        
        // Initialize status signals
        bottomVelocity = bottomMotor.getVelocity();
        topVelocity = topMotor.getVelocity();
        bottomAppliedVolts = bottomMotor.getMotorVoltage();
        topAppliedVolts = topMotor.getMotorVoltage();
        bottomSupplyCurrent = bottomMotor.getSupplyCurrent();
        topSupplyCurrent = topMotor.getSupplyCurrent();
        bottomStatorCurrent = bottomMotor.getStatorCurrent();
        topStatorCurrent = topMotor.getStatorCurrent();
        bottomTemp = bottomMotor.getDeviceTemp();
        topTemp = topMotor.getDeviceTemp();
        
        // Optimize bus utilization by setting update frequencies
        // High frequency for velocity (needed for control)
        bottomVelocity.setUpdateFrequency(100);
        topVelocity.setUpdateFrequency(100);
        
        // Lower frequency for logging signals
        bottomAppliedVolts.setUpdateFrequency(50);
        topAppliedVolts.setUpdateFrequency(50);
        bottomSupplyCurrent.setUpdateFrequency(50);
        topSupplyCurrent.setUpdateFrequency(50);
        bottomStatorCurrent.setUpdateFrequency(50);
        topStatorCurrent.setUpdateFrequency(50);
        bottomTemp.setUpdateFrequency(4);
        topTemp.setUpdateFrequency(4);
    }
    
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        // Refresh all status signals
        bottomVelocity.refresh();
        topVelocity.refresh();
        bottomAppliedVolts.refresh();
        topAppliedVolts.refresh();
        bottomSupplyCurrent.refresh();
        topSupplyCurrent.refresh();
        bottomStatorCurrent.refresh();
        topStatorCurrent.refresh();
        bottomTemp.refresh();
        topTemp.refresh();
        
        // Update inputs with current values
        inputs.bottomVelocityRPS = bottomVelocity.getValueAsDouble();
        inputs.topVelocityRPS = topVelocity.getValueAsDouble();
        inputs.bottomAppliedVolts = bottomAppliedVolts.getValueAsDouble();
        inputs.topAppliedVolts = topAppliedVolts.getValueAsDouble();
        inputs.bottomSupplyCurrentAmps = bottomSupplyCurrent.getValueAsDouble();
        inputs.topSupplyCurrentAmps = topSupplyCurrent.getValueAsDouble();
        inputs.bottomStatorCurrentAmps = bottomStatorCurrent.getValueAsDouble();
        inputs.topStatorCurrentAmps = topStatorCurrent.getValueAsDouble();
        inputs.bottomTempCelsius = bottomTemp.getValueAsDouble();
        inputs.topTempCelsius = topTemp.getValueAsDouble();
        inputs.targetBottomVelocityRPS = targetBottomVelocity;
        inputs.targetTopVelocityRPS = targetTopVelocity;
    }
    
    @Override
    public void setVelocity(double bottomVelocityRPS) {
        targetBottomVelocity = bottomVelocityRPS;
        targetTopVelocity = bottomVelocityRPS * 3.0; // Top wheel spins 3x faster due to gear ratio
        
        // Only set bottom motor (leader) to target velocity
        // Top motor (follower) will automatically follow at 3x speed due to configured gear ratio
        bottomMotor.setControl(velocityRequest.withVelocity(bottomVelocityRPS));
    }
    
    @Override
    public void stop() {
        targetBottomVelocity = 0.0;
        targetTopVelocity = 0.0;
        bottomMotor.stopMotor();
        topMotor.stopMotor();
    }
    
    @Override
    public void setVoltage(double volts) {
        targetBottomVelocity = 0.0;
        targetTopVelocity = 0.0;
        bottomMotor.setControl(voltageRequest.withOutput(volts));
        topMotor.setControl(voltageRequest.withOutput(volts));
    }
}
