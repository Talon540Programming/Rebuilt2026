package frc.robot.subsystems.Climberz;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ClimberConstants;

public class ClimberzIOKraken implements ClimberzIO {
    
    private final TalonFX leaderMotor;
    private final TalonFX followerMotor;
    
    // Status signals - Leader
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> leaderCurrent;
    private final StatusSignal<Temperature> leaderTemp;
    
    // Status signals - Follower
    private final StatusSignal<Current> followerCurrent;
    private final StatusSignal<Temperature> followerTemp;
    
    // Control request
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    
    public ClimberzIOKraken() {
        leaderMotor = new TalonFX(ClimberConstants.kLeaderMotorId);
        followerMotor = new TalonFX(ClimberConstants.kFollowerMotorId);
        
        // Configure leader motor
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.CurrentLimits.StatorCurrentLimit = 80;
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leaderConfig.CurrentLimits.SupplyCurrentLimit = 60;
        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leaderMotor.getConfigurator().apply(leaderConfig);
        
        // Configure follower motor
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.CurrentLimits.StatorCurrentLimit = 80;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        followerConfig.CurrentLimits.SupplyCurrentLimit = 60;
        followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerMotor.getConfigurator().apply(followerConfig);
        
        // Set follower to follow leader (same direction - both on same mechanism)
        followerMotor.setControl(new Follower(ClimberConstants.kLeaderMotorId, MotorAlignmentValue.Aligned));
        
        // Get status signals - Leader
        position = leaderMotor.getPosition();
        velocity = leaderMotor.getVelocity();
        appliedVolts = leaderMotor.getMotorVoltage();
        leaderCurrent = leaderMotor.getStatorCurrent();
        leaderTemp = leaderMotor.getDeviceTemp();
        
        // Get status signals - Follower (just current and temp for monitoring)
        followerCurrent = followerMotor.getStatorCurrent();
        followerTemp = followerMotor.getDeviceTemp();
        
        // Set update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            position, velocity, appliedVolts, leaderCurrent, leaderTemp,
            followerCurrent, followerTemp
        );
        
        // Optimize bus utilization
        leaderMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs(ClimberzIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, leaderCurrent, leaderTemp,
            followerCurrent, followerTemp
        );
        
        inputs.positionRotations = position.getValueAsDouble();
        inputs.velocityRotPerSec = velocity.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.leaderCurrentAmps = leaderCurrent.getValueAsDouble();
        inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();
        inputs.leaderTempCelsius = leaderTemp.getValueAsDouble();
        inputs.followerTempCelsius = followerTemp.getValueAsDouble();
    }
    
    @Override
    public void setDutyCycle(double dutyCycle) {
        leaderMotor.setControl(dutyCycleControl.withOutput(dutyCycle));
    }
    
    @Override
    public void stop() {
        leaderMotor.setControl(dutyCycleControl.withOutput(0));
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        leaderMotor.setNeutralMode(mode);
        followerMotor.setNeutralMode(mode);
    }
}