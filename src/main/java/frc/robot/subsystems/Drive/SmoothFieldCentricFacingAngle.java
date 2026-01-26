package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.HeadingPID;
import frc.robot.utility.ShootingCalculator;

/**
 * Custom swerve request that smoothly tracks a target point on the field.
 * Calculates heading internally at 250Hz (synced with odometry) to eliminate
 * the 4ms/20ms timing mismatch that causes module jitter.
 */
public class SmoothFieldCentricFacingAngle implements SwerveRequest {

    // Underlying field centric request for composition
    private final SwerveRequest.FieldCentric m_fieldCentric = new SwerveRequest.FieldCentric();

    // Heading PID controller
    public final PIDController HeadingController = new PIDController(5.0, 0.0, 0.0);

    // Target point to face (set externally based on alliance)
    private Translation2d m_redTarget = new Translation2d();
    private Translation2d m_blueTarget = new Translation2d();
    // Dynamic virtual target (overrides static targets when non-null)
    private Translation2d virtualTarget = null;

    // For tracking rate of change of target heading
    private double m_previousTimestamp = 0.0;

    // Request parameters
    public double velocityX = 0.0;
    public double velocityY = 0.0;
    public double deadband = 0.0;
    public double rotationalDeadband = 0.0;
    public SwerveModule.DriveRequestType driveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    public SwerveModule.SteerRequestType steerRequestType = SwerveModule.SteerRequestType.MotionMagicExpo;
    public ForwardPerspectiveValue forwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

    public SmoothFieldCentricFacingAngle() {
        HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        
        // Get current pose from the 250Hz odometry thread
        Pose2d currentPose = parameters.currentPose;
        
       // Use virtual target if set, otherwise use alliance-based static target
        Translation2d target;
        if (virtualTarget != null) {
            target = virtualTarget;
        } else {
            target = m_blueTarget;
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                target = m_redTarget;
            }
        }
        
        // Calculate target heading at 250Hz
        // Calculate target heading at 250Hz using shooter position offset
        double targetRadians = ShootingCalculator.calculateAimingHeading(currentPose, target) + HeadingPID.shooterThetaOffset.get();
        
        // Don't apply operator perspective rotation to heading target
        // The heading should be absolute field-relative (always face the hub the same way)
        double adjustedTargetRadians = targetRadians;

        // Calculate target rate feedforward from how fast the target is changing
        double targetRateFeedforward = 0.0;

        // Store for next iteration
        m_previousTimestamp = parameters.timestamp;

        // Calculate rotation rate from PID + feedforward
        double toApplyOmega = targetRateFeedforward + HeadingController.calculate(
            currentPose.getRotation().getRadians(),
            adjustedTargetRadians
        );

        Logger.recordOutput("SmoothHeading/TargetRadians", adjustedTargetRadians);
        Logger.recordOutput("SmoothHeading/CurrentRadians", currentPose.getRotation().getRadians());
        Logger.recordOutput("SmoothHeading/Feedforward", targetRateFeedforward);
        Logger.recordOutput("SmoothHeading/PIDOutput", HeadingController.calculate(currentPose.getRotation().getRadians(), adjustedTargetRadians));
        Logger.recordOutput("SmoothHeading/TotalOmega", toApplyOmega);
        Logger.recordOutput("SmoothHeading/DeltaTime", parameters.timestamp - m_previousTimestamp);

        // Use the underlying FieldCentric request
        return m_fieldCentric
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withRotationalRate(toApplyOmega)
            .withDeadband(deadband)
            .withRotationalDeadband(rotationalDeadband)
            .withDriveRequestType(driveRequestType)
            .withSteerRequestType(steerRequestType)
            .withForwardPerspective(forwardPerspective)
            .apply(parameters, modulesToApply);

    
    }

    // Fluent API methods
    public SmoothFieldCentricFacingAngle withVelocityX(double velocityX) {
        this.velocityX = velocityX;
        return this;
    }

    public SmoothFieldCentricFacingAngle withVelocityY(double velocityY) {
        this.velocityY = velocityY;
        return this;
    }

    public SmoothFieldCentricFacingAngle withRedTarget(Translation2d target) {
        this.m_redTarget = target;
        return this;
    }

    public SmoothFieldCentricFacingAngle withBlueTarget(Translation2d target) {
        this.m_blueTarget = target;
        return this;
    }

    public SmoothFieldCentricFacingAngle withTargets(Translation2d redTarget, Translation2d blueTarget) {
        this.m_redTarget = redTarget;
        this.m_blueTarget = blueTarget;
        return this;
    }

    public SmoothFieldCentricFacingAngle withDeadband(double deadband) {
        this.deadband = deadband;
        return this;
    }

    public SmoothFieldCentricFacingAngle withRotationalDeadband(double rotationalDeadband) {
        this.rotationalDeadband = rotationalDeadband;
        return this;
    }

    public SmoothFieldCentricFacingAngle withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
        this.driveRequestType = driveRequestType;
        return this;
    }

    public SmoothFieldCentricFacingAngle withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
        this.steerRequestType = steerRequestType;
        return this;
    }

    public SmoothFieldCentricFacingAngle withForwardPerspective(ForwardPerspectiveValue forwardPerspective) {
        this.forwardPerspective = forwardPerspective;
        return this;
    }

    public SmoothFieldCentricFacingAngle withVirtualTarget(Translation2d virtualTarget) {
        this.virtualTarget = virtualTarget;
        return this;
    }

    public void clearVirtualTarget() {
        this.virtualTarget = null;
    }
}