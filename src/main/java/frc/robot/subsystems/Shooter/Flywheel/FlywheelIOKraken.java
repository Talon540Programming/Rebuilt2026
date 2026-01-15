package frc.robot.subsystems.Shooter.Flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.Shooter.ShooterConstants;


public class FlywheelIOKraken implements FlywheelIO {
    
    // Two X60 motors on same mechanism (belt connects main to top flywheel)
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    
    // Status signals (from leader only)
    private final StatusSignal<AngularVelocity> velocityRotPerSec;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> currentAmps;
    private final StatusSignal<Temperature> tempCelsius;
    
    // Control requests
    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    
    private double targetVelocityRPM = 0.0;
    
    // Velocity tolerance for "at setpoint" check (RPM)
    private static final double kVelocityTolerance = 50.0;
    
    public FlywheelIOKraken() {
        leaderMotor = new TalonFX(ShooterConstants.kFlywheelMotor1Id);
        followerMotor = new TalonFX(ShooterConstants.kFlywheelMotor2Id);
        
        // Configure leader motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Current limits - X60 can handle high current
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Motor direction
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Coast for flywheel
        
        // Velocity PID - slot 0
        config.Slot0.kP = ShooterConstants.kFlywheelkP;
        config.Slot0.kI = ShooterConstants.kFlywheelkI;
        config.Slot0.kD = ShooterConstants.kFlywheelkD;
        config.Slot0.kV = ShooterConstants.kFlywheelkV;
        
        leaderMotor.getConfigurator().apply(config);
        
        // Configure follower - inverted relative to leader (opposite spin direction)
        // Belt connects main flywheel to top flywheel, top spins opposite
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.CurrentLimits.StatorCurrentLimit = 80;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        followerConfig.CurrentLimits.SupplyCurrentLimit = 60;
        followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Opposite direction
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        followerMotor.getConfigurator().apply(followerConfig);
        
        // Set follower to follow leader (with opposite direction already configured)
        followerMotor.setControl(new Follower(ShooterConstants.kFlywheelMotor1Id, MotorAlignmentValue.Aligned));
        
        // Get status signals from leader
        velocityRotPerSec = leaderMotor.getVelocity();
        appliedVolts = leaderMotor.getMotorVoltage();
        currentAmps = leaderMotor.getStatorCurrent();
        tempCelsius = leaderMotor.getDeviceTemp();
        
        // Set update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            velocityRotPerSec,
            appliedVolts,
            currentAmps,
            tempCelsius
        );
        
        // Optimize bus utilization
        leaderMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            velocityRotPerSec,
            appliedVolts,
            currentAmps,
            tempCelsius
        );
        
        inputs.velocityRotPerSec = velocityRotPerSec.getValueAsDouble();
        inputs.velocityRPM = inputs.velocityRotPerSec * 60.0;
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = currentAmps.getValueAsDouble();
        inputs.tempCelsius = tempCelsius.getValueAsDouble();
        inputs.targetVelocityRPM = targetVelocityRPM;
        inputs.atSetpoint = Math.abs(inputs.velocityRPM - targetVelocityRPM) < kVelocityTolerance;
    }
    
    @Override
    public void setVelocity(double velocityRPM) {
        targetVelocityRPM = velocityRPM;
        double velocityRotPerSec = velocityRPM / 60.0;
        leaderMotor.setControl(velocityControl.withVelocity(velocityRotPerSec));
    }
    
    @Override
    public void setDutyCycle(double dutyCycle) {
        targetVelocityRPM = 0.0;
        leaderMotor.setControl(dutyCycleControl.withOutput(dutyCycle));
    }
    
    @Override
    public void stop() {
        targetVelocityRPM = 0.0;
        leaderMotor.setControl(dutyCycleControl.withOutput(0));
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        leaderMotor.setNeutralMode(mode);
        followerMotor.setNeutralMode(mode);
    }
}