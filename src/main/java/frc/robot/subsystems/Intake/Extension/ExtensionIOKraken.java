package frc.robot.subsystems.Intake.Extension;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake.IntakeConstants;

import com.ctre.phoenix6.controls.MotionMagicVoltage;


public class ExtensionIOKraken implements ExtensionIO {
    
    private final TalonFX pivot;
    
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> appliedVolts;
    private final StatusSignal<Current> current;
    private final StatusSignal<Temperature> temp;
    
    private final MotionMagicVoltage mmRequest = new MotionMagicVoltage(0.0);

    public ExtensionIOKraken() {
        pivot = new TalonFX(IntakeConstants.extensionMotorId, TunerConstants.kCANBus);
        
        TalonFXConfiguration config = new TalonFXConfiguration();
        
        // Current limits for X44
        config.CurrentLimits.StatorCurrentLimit = 40.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Motor direction - adjust based on your mechanism
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; //TODO need to tune for real bot
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Scale rotor -> mechanism rotations
        config.Feedback.SensorToMechanismRatio = IntakeConstants.extensionSensorToMechanismRatio.get(); //TODO need to tune for real bot

        // Slot0 PID
        config.Slot0.kP = IntakeConstants.extensionkP.get();
        config.Slot0.kI = IntakeConstants.extensionkI.get();
        config.Slot0.kD = IntakeConstants.extensionkD.get();
        config.Slot0.kS = IntakeConstants.extensionkS.get();
        config.Slot0.kV = IntakeConstants.extensionkV.get();

        // Software limits to prevent over-travel
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.extensionDeployedPosRot.get() + 0.05;  // Small buffer
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.extensionStowedPosRot.get() - 0.05;

        // Motion Magic constraints:
        config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.extensionMMCruiseVelRotPerSec.get();
        config.MotionMagic.MotionMagicAcceleration = IntakeConstants.extensionMMAccelRotPerSec2.get();
        config.MotionMagic.MotionMagicJerk = IntakeConstants.extensionMMJerkRotPerSec3.get();

        pivot.getConfigurator().apply(config);

        
        // Get status signals
        position = pivot.getPosition();
        velocity = pivot.getVelocity();
        appliedVolts = pivot.getMotorVoltage();
        current = pivot.getStatorCurrent();
        temp = pivot.getDeviceTemp();
        
        // Set update frequencies
        BaseStatusSignal.setUpdateFrequencyForAll(50, position, velocity, appliedVolts, current, temp);
        pivot.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs(PivotIOInputs inputs) {
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current, temp);
        
        inputs.positionRotations = position.getValueAsDouble();
        inputs.velocityRotPerSec = velocity.getValueAsDouble();
        inputs.appliedVolts = appliedVolts.getValueAsDouble();
        inputs.currentAmps = current.getValueAsDouble();
        inputs.tempCelsius = temp.getValueAsDouble();
    }
    
    @Override
    public void stop() {
        pivot.stopMotor();
    }
    
    @Override
    public void setBrakeMode(boolean brake) {
        pivot.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void runMotionMagicPosition(double positionRotations, double feedForwardVolts) {
        pivot.setControl(
            mmRequest.withPosition(positionRotations)
                    .withFeedForward(feedForwardVolts)
        );
    }

      @Override
        public void setPosition(double positionRotations) {
            // Sets the integrated sensor position. This value affects subsequent position reads.
            pivot.setPosition(positionRotations);
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        pivot.set(dutyCycle);
    }


}