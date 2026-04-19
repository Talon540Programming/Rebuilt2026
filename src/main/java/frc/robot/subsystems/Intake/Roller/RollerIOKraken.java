package frc.robot.subsystems.Intake.Roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake.IntakeConstants;


public class RollerIOKraken implements RollerIO {
    
    private final TalonFX rollerLeader;
    private final TalonFX rollerFollower;
    
    // Leader status signals
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> current;
    private final StatusSignal<Temperature> temp;
    
    // Follower status signals
    private final StatusSignal<Current> followerCurrent;
    private final StatusSignal<Temperature> followerTemp;
    
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
    
    public RollerIOKraken() {
        rollerLeader = new TalonFX(IntakeConstants.rollerMotor1Id, TunerConstants.kCANBus);
        rollerFollower = new TalonFX(IntakeConstants.rollerMotor2Id, TunerConstants.kCANBus);
        
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        
        // Current limits for Kraken X60 (on separate 40A breakers)
        leaderConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        leaderConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Motor direction - both spin clockwise
        leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        rollerLeader.getConfigurator().apply(leaderConfig);
        
        // Follower config - same current limits, same direction
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.CurrentLimits.StatorCurrentLimit = 80.0;
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        followerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        rollerFollower.getConfigurator().apply(followerConfig);
        
        // Set follower to strictly follow the leader (same direction since both spin clockwise)
        rollerFollower.setControl(new Follower(IntakeConstants.rollerMotor1Id, MotorAlignmentValue.Opposed));
        
        // Get leader status signals
        velocity = rollerLeader.getVelocity();
        appliedVolts = rollerLeader.getMotorVoltage();
        current = rollerLeader.getStatorCurrent();
        temp = rollerLeader.getDeviceTemp();
        
        // Get follower status signals
        followerCurrent = rollerFollower.getStatorCurrent();
        followerTemp = rollerFollower.getDeviceTemp();
        
        // Set update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(50, velocity, appliedVolts, current, temp, followerCurrent, followerTemp);
        rollerLeader.optimizeBusUtilization();
        rollerFollower.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs(RollerIOInputs inputs) {
        BaseStatusSignal.refreshAll(velocity, appliedVolts, current, temp, followerCurrent, followerTemp);
        
        inputs.velocityRotPerSec = velocity.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
        
        // Follower motor data
        inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();
        inputs.followerTempCelsius = followerTemp.getValueAsDouble();
    }
    
    @Override
    public void setDutyCycle(double dutyCycle) {
        // Only command the leader - follower automatically follows
        rollerLeader.setControl(dutyCycleControl.withOutput(dutyCycle));
    }
    
    @Override
    public void stop() {
        rollerLeader.stopMotor();
        // Follower will stop automatically since it follows the leader
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        rollerLeader.setNeutralMode(mode);
        rollerFollower.setNeutralMode(mode);
    }
}