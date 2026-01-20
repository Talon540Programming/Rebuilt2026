package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
    private Translation2d m_virtualTarget = null;

    // For tracking rate of change of target heading
    private double m_previousTimestamp = 0.0;

    // Request parameters
    public double VelocityX = 0.0;
    public double VelocityY = 0.0;
    public double Deadband = 0.0;
    public double RotationalDeadband = 0.0;
    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagicExpo;
    public ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue.OperatorPerspective;

    public SmoothFieldCentricFacingAngle() {
        HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public StatusCode apply(SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
        
        // Get current pose from the 250Hz odometry thread
        Pose2d currentPose = parameters.currentPose;
        
       // Use virtual target if set, otherwise use alliance-based static target
        Translation2d target;
        if (m_virtualTarget != null) {
            target = m_virtualTarget;
        } else {
            target = m_blueTarget;
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                target = m_redTarget;
            }
        }
        
        // Calculate target heading at 250Hz
        double dx = target.getX() - currentPose.getX();
        double dy = target.getY() - currentPose.getY();
        double targetRadians = Math.atan2(dy, dx);
        
        Rotation2d angleToFace = new Rotation2d(targetRadians);
        if (ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
            angleToFace = angleToFace.rotateBy(parameters.operatorForwardDirection);
        }
        double adjustedTargetRadians = angleToFace.getRadians();

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
            .withVelocityX(VelocityX)
            .withVelocityY(VelocityY)
            .withRotationalRate(toApplyOmega)
            .withDeadband(Deadband)
            .withRotationalDeadband(RotationalDeadband)
            .withDriveRequestType(DriveRequestType)
            .withSteerRequestType(SteerRequestType)
            .withForwardPerspective(ForwardPerspective)
            .apply(parameters, modulesToApply);

    
    }

    // Fluent API methods
    public SmoothFieldCentricFacingAngle withVelocityX(double velocityX) {
        this.VelocityX = velocityX;
        return this;
    }

    public SmoothFieldCentricFacingAngle withVelocityY(double velocityY) {
        this.VelocityY = velocityY;
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
        this.Deadband = deadband;
        return this;
    }

    public SmoothFieldCentricFacingAngle withRotationalDeadband(double rotationalDeadband) {
        this.RotationalDeadband = rotationalDeadband;
        return this;
    }

    public SmoothFieldCentricFacingAngle withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
        this.DriveRequestType = driveRequestType;
        return this;
    }

    public SmoothFieldCentricFacingAngle withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
        this.SteerRequestType = steerRequestType;
        return this;
    }

    public SmoothFieldCentricFacingAngle withForwardPerspective(ForwardPerspectiveValue forwardPerspective) {
        this.ForwardPerspective = forwardPerspective;
        return this;
    }

    public SmoothFieldCentricFacingAngle withVirtualTarget(Translation2d virtualTarget) {
        this.m_virtualTarget = virtualTarget;
        return this;
    }

    public void clearVirtualTarget() {
        this.m_virtualTarget = null;
    }
}