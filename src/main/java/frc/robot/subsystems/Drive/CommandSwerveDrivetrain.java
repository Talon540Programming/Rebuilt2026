// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;


import org.littletonrobotics.junction.Logger;


import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;


/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

       /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
        public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    /**
     * Configures PathPlanner's AutoBuilder for path following.
     */
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder: " + ex.getMessage(), ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // Log current pose for AdvantageScope visualization
        Logger.recordOutput("Odometry/Robot", getPose());
    }

    /**
     * Gets the current pose of the robot.
     *
     * @return Current pose
     */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /**
     * Gets the current heading of the robot.
     *
     * @return Current heading as Rotation2d
     */
    public Rotation2d getHeading() {
        return getState().Pose.getRotation();
    }

    /**
     * Gets the field-relative velocity of the robot.
     *
     * @return Field-relative ChassisSpeeds
     */
    public ChassisSpeeds getFieldVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            getState().Speeds,
            getHeading()
        );
    }

    /**
     * Gets the magnitude of the robot's velocity.
     *
     * @return Velocity magnitude
     */
    public LinearVelocity getVelocityMagnitude() {
        ChassisSpeeds cs = getFieldVelocity();
        return MetersPerSecond.of(Math.hypot(cs.vxMetersPerSecond, cs.vyMetersPerSecond));
    }

    /**
     * Resets the field-centric heading to the current heading.
     * This is useful for resetting the zero point of the gyro.
     */
    /* 
    public void seedFieldCentric() {
        resetRotation(new Rotation2d());
    }
  */
  //this is so 0 is now facing the opposite alliance station revert to above code for default control 
  public void seedFieldCentric() {
    var alliance = DriverStation.getAlliance();
    
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
        resetRotation(Rotation2d.k180deg);
    } else {
        resetRotation(new Rotation2d());
    }
}
    /**
 * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
 * while still accounting for measurement noise.
 *
 * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
 * @param timestampSeconds The timestamp of the vision measurement in seconds.
 */
@Override
public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
    super.addVisionMeasurement(visionRobotPoseMeters, com.ctre.phoenix6.Utils.fpgaToCurrentTime(timestampSeconds));
}

/**
 * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
 * while still accounting for measurement noise.
 *
 * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
 * @param timestampSeconds The timestamp of the vision measurement in seconds.
 * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
 *     in the form [x, y, theta]ᵀ, with units in meters and radians.
 */
@Override
public void addVisionMeasurement(
    Pose2d visionRobotPoseMeters,
    double timestampSeconds,
    Matrix<N3, N1> visionMeasurementStdDevs
) {
    super.addVisionMeasurement(visionRobotPoseMeters, com.ctre.phoenix6.Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
}

    /* Simulation support */
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Thread m_simThread;
    private double m_lastSimTime;

    private void startSimThread() {
        m_lastSimTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        m_simThread = new Thread(() -> {
            while (true) {
                var currentTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
                double deltaTime = currentTime - m_lastSimTime;
                m_lastSimTime = currentTime;

                // Use the measured time delta, get battery voltage from WPILib
                updateSimState(deltaTime, RobotController.getBatteryVoltage());

                try {
                    Thread.sleep((long) (kSimLoopPeriod * 1000));
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        });
        m_simThread.setDaemon(true);
        m_simThread.start();
    }
}