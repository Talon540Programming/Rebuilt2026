package frc.robot.subsystems.Shooter.Flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
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
    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);
    
    private double targetVelocityRPM = 0.0;
    
    
    public FlywheelIOKraken() {
        leaderMotor = new TalonFX(ShooterConstants.flywheelMotor1Id, TunerConstants.kCANBus);
        followerMotor = new TalonFX(ShooterConstants.flywheelMotor2Id, TunerConstants.kCANBus);
        
        // Configure leader motor
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Current limits
        config.CurrentLimits.StatorCurrentLimit = ShooterConstants.flywheelStatorCurrentLimit.get();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.flywheelSupplyCurrentLimit.get();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Motor direction - both spin same way, configure on robot
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO need to tune for real bot
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Slot 0 gains for Motion Magic Velocity
        config.Slot0.kP = ShooterConstants.flywheelkP.get();
        config.Slot0.kI = ShooterConstants.flywheelkI.get();
        config.Slot0.kD = ShooterConstants.flywheelkD.get();
        config.Slot0.kS = ShooterConstants.flywheelkS.get();
        config.Slot0.kV = ShooterConstants.flywheelkV.get();
        config.Slot0.kA = ShooterConstants.flywheelkA.get();
        
        // Motion Magic Velocity settings (no CruiseVelocity for velocity control)
        config.MotionMagic.MotionMagicAcceleration = ShooterConstants.flywheelMMAccelRotPerSec2.get();
        config.MotionMagic.MotionMagicJerk = ShooterConstants.flywheelMMJerkRotPerSec3.get();
        
        leaderMotor.getConfigurator().apply(config);
        
        // Configure follower - same direction as leader
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.flywheelStatorCurrentLimit.get();
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        followerConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.flywheelSupplyCurrentLimit.get();
        followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;  // Same direction
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        followerMotor.getConfigurator().apply(followerConfig);
        
        // Set follower to follow leader (Aligned = same direction)
        followerMotor.setControl(new Follower(ShooterConstants.flywheelMotor1Id, MotorAlignmentValue.Aligned));  // false = same direction

        
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
        inputs.atSetpoint = Math.abs(inputs.velocityRPM - targetVelocityRPM) < ShooterConstants.flywheelVelToleranceRPM.get();
    }
    
    @Override
    public void setVelocity(double velocityRPM) {
        targetVelocityRPM = velocityRPM;
        double velocityRotPerSec = velocityRPM / 60.0;
        // Motion Magic Velocity uses rotations per second
        leaderMotor.setControl(velocityControl.withVelocity(velocityRotPerSec));
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        NeutralModeValue mode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        leaderMotor.setNeutralMode(mode);
        followerMotor.setNeutralMode(mode);
    }

    @Override
    public void stop() {
        leaderMotor.stopMotor();
        targetVelocityRPM = 0.0;
    }
}