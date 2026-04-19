package frc.robot.subsystems.Shooter.Flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
    
    // Motion Magic Velocity control request
    private final MotionMagicVelocityVoltage motionMagicVelocity = new MotionMagicVelocityVoltage(0);
    
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
        
        // Motor direction
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // Flywheel can only spin forward, no reverse
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = 0.0;
        
        // Motion Magic Velocity PID gains (Slot 0)
        // kS: Static friction - voltage to overcome friction at 0 velocity
        // kV: Velocity feedforward - voltage per rotation per second
        // kP: Proportional gain - correction for velocity error
        // kI, kD: Usually 0 for flywheels
        config.Slot0.kS = ShooterConstants.flywheelkS.get();
        config.Slot0.kV = ShooterConstants.flywheelkV.get();
        config.Slot0.kP = ShooterConstants.flywheelkP.get();
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kA = 0.0;
        
        // Motion Magic Velocity configuration
        // Acceleration: How fast flywheel can change velocity (rot/s^2)
        // Set very high for maximum acceleration (as fast as possible)
        config.MotionMagic.MotionMagicAcceleration = ShooterConstants.flywheelMMAccelRotPerSec2.get();
        // Jerk: 0 = no jerk limit (trapezoidal profile)
        config.MotionMagic.MotionMagicJerk = 0;
        
        leaderMotor.getConfigurator().apply(config);
        
        // Configure follower - same direction as leader
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.flywheelStatorCurrentLimit.get();
        followerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        followerConfig.CurrentLimits.SupplyCurrentLimit = ShooterConstants.flywheelSupplyCurrentLimit.get();
        followerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        followerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        followerMotor.getConfigurator().apply(followerConfig);
        
        // Set follower to follow leader (Aligned = same direction)
        followerMotor.setControl(new Follower(ShooterConstants.flywheelMotor1Id, MotorAlignmentValue.Aligned));
        
        // Configure Motion Magic Velocity request
        // EnableFOC: true for better performance (~15% more power)
        motionMagicVelocity.EnableFOC = true;
        motionMagicVelocity.Slot = 0;
        
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
    
    @Override
    public void runVelocity(double velocityRPM) {
        targetVelocityRPM = velocityRPM;
        double velocityRotPerSecTarget = velocityRPM / 60.0;
        
        if (velocityRPM <= 0) {
            leaderMotor.stopMotor();
        } else {
            // Use Motion Magic Velocity for smooth acceleration to target
            leaderMotor.setControl(motionMagicVelocity.withVelocity(velocityRotPerSecTarget));
        }
    }
}
