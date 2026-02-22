package frc.robot.subsystems.Shooter.Flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
    
    // Control requests - MA style bang-bang using motor's internal 1kHz loop
    private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrent = new VelocityTorqueCurrentFOC(0);
    
    // Bang-bang state machine
    private double targetVelocityRPM = 0.0;
    private FlywheelState currentState = FlywheelState.COAST;
    private Debouncer idleDebouncer;
    private long shotCount = 0;
    
    
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
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO Tuned Cad
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        
        // MA-style bang-bang configuration
        // Extremely high kP makes velocity control act as bang-bang
        // Motor's internal 1kHz loop handles the on/off switching
        config.Slot0.kP = 999999.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kS = 0.0;
        config.Slot0.kV = 0.0;
        config.Slot0.kA = 0.0;
        
        // Limit peak outputs for bang-bang behavior
        // Duty cycle mode: full forward, no reverse
        config.MotorOutput.PeakForwardDutyCycle = 1.0;
        config.MotorOutput.PeakReverseDutyCycle = 0.0;
        
        // Torque current mode: limited forward current, no reverse
        config.TorqueCurrent.PeakForwardTorqueCurrent = ShooterConstants.flywheelBangBangTorqueCurrent.get();
        config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
        
        leaderMotor.getConfigurator().apply(config);
        
        // Initialize debouncer for state transitions to IDLE
        idleDebouncer = new Debouncer(
            ShooterConstants.flywheelBangBangDebounceSeconds.get(),
            DebounceType.kRising
        );
        
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
        inputs.state = currentState;
        inputs.shotCount = shotCount;
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
        currentState = FlywheelState.COAST;
    }
    
    @Override
    public void runBangBang(double velocityRPM) {
        targetVelocityRPM = velocityRPM;
        double velocityRotPerSecTarget = velocityRPM / 60.0;
        
        // Calculate if we're within tolerance
        double currentRPM = velocityRotPerSec.getValueAsDouble() * 60.0;
        double error = Math.abs(currentRPM - velocityRPM);
        boolean inTolerance = error <= ShooterConstants.flywheelTorqueCurrentTolerance.get();
        boolean readyForIdle = idleDebouncer.calculate(inTolerance);
                
        switch (currentState) {
            case COAST:
                if (velocityRPM > 0) {
                    currentState = FlywheelState.STARTUP;
                }
                break;
                
            case STARTUP:
                if (velocityRPM <= 0) {
                    currentState = FlywheelState.COAST;
                } else if (readyForIdle) {
                    currentState = FlywheelState.IDLE;
                }
                break;
                
            case IDLE:
                if (velocityRPM <= 0) {
                    currentState = FlywheelState.COAST;
                } else if (!inTolerance) {
                    currentState = FlywheelState.RECOVERY;
                    shotCount++;
                }
                break;
                
            case RECOVERY:
                if (velocityRPM <= 0) {
                    currentState = FlywheelState.COAST;
                } else if (readyForIdle) {
                    currentState = FlywheelState.IDLE;
                }
                break;
        }
        
        // Apply control based on current state
        switch (currentState) {
            case COAST:
                leaderMotor.stopMotor();
                break;
                
            case STARTUP:
            case RECOVERY:
                // Max duty cycle for fastest spinup/recovery
                leaderMotor.setControl(velocityDutyCycle.withVelocity(velocityRotPerSecTarget));
                break;
                
            case IDLE:
                // Constant torque current for consistent ball exit velocity
                leaderMotor.setControl(velocityTorqueCurrent.withVelocity(velocityRotPerSecTarget));
                break;
        }
    }
}