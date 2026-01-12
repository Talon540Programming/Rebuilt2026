// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.Telemetry;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.DriveToPose;
import frc.robot.subsystems.Drive.SetReefCenterHeading;
import frc.robot.subsystems.Drive.SetReefSideHeading;
import frc.robot.subsystems.Vision.VisionBase;
import frc.robot.subsystems.Vision.VisionIOLimelight;
import edu.wpi.first.math.filter.SlewRateLimiter;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.generated.TunerConstants;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final VisionIOLimelight visionIO = new VisionIOLimelight();
    private final VisionBase vision = new VisionBase(visionIO, drivetrain);
    private DriveToPose driveToPose = new DriveToPose(drivetrain, vision);
    private final SetReefSideHeading autoHeading = new SetReefSideHeading(vision);
    private final SetReefCenterHeading faceReefCenter = new SetReefCenterHeading(vision);

    private final SendableChooser<Command> autoChooser;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(4);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(4);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(4);

    private static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(edu.wpi.first.units.Units.MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  
    private final SwerveRequest.FieldCentricFacingAngle headingDrive = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(0)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);
  
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        configureBindings();
        
        autoChooser = AutoBuilder.buildAutoChooser("default auto"); //pick a default
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

  
    private void configureBindings() {
        m_driverController.start().onTrue(Commands.runOnce(()->{
            drivetrain.seedFieldCentric();
            vision.setGyroInitialized();
        }));
        m_driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));

        // Y button toggles face reef center - also disables auto heading if it's on
        m_driverController.povRight().onTrue(Commands.runOnce(() -> {
            if (autoHeading.isEnabled()) {
                autoHeading.disableAutoHeading();
            }
            faceReefCenter.toggleFaceReef();
        }));

        // Update the existing povDown binding to also disable face reef
        m_driverController.povDown().onTrue(Commands.runOnce(() -> {
            if (faceReefCenter.isEnabled()) {
                faceReefCenter.disableFaceReef();
            }
            autoHeading.toggleAutoHeading();
        }));

        m_driverController.povUp().whileTrue(
            (driveToPose.createReefPathCommand(DriveToPose.Side.Middle).until(() -> driveToPose.haveReefConditionsChanged()).repeatedly()));

        m_driverController.leftBumper().whileTrue(
            (driveToPose.createReefPathCommand(DriveToPose.Side.Left).until(() -> driveToPose.haveReefConditionsChanged()).repeatedly()));

        m_driverController.rightBumper().whileTrue(
            (driveToPose.createReefPathCommand(DriveToPose.Side.Right).until(() -> driveToPose.haveReefConditionsChanged()).repeatedly()));

        m_driverController.leftTrigger().whileTrue(
            (driveToPose.createStationPathCommand().until(() -> driveToPose.haveStationConditionsChanged()).repeatedly()));

        headingDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        headingDrive.HeadingController.setPID(11, 0.1, 0.5);

        drivetrain.setDefaultCommand(
            drivetrain.run(() -> {
                // Get raw joystick inputs
                double rawX = -m_driverController.getLeftY();
                double rawY = -m_driverController.getLeftX();
                double rawRot = -m_driverController.getRightX();
                
                // Apply deadband BEFORE the limiter
                rawX = Math.abs(rawX) > OperatorConstants.deadband ? rawX : 0.0;
                rawY = Math.abs(rawY) > OperatorConstants.deadband ? rawY : 0.0;
                rawRot = Math.abs(rawRot) > OperatorConstants.deadband ? rawRot : 0.0;
                
                // Now apply slew rate limiting
                double xSpeed = xLimiter.calculate(rawX);
                double ySpeed = yLimiter.calculate(rawY);
                double rotSpeed = rotLimiter.calculate(rawRot);
                
                // Check if driver is manually rotating - this always takes priority
                boolean driverRotating = autoHeading.isDriverRotating(rotSpeed);
                boolean atSetpoint = driveToPose.isAtSetpoint(0.3);
                
                // Determine which auto-heading mode to use (if any)
                boolean useAutoHeading = autoHeading.isEnabled() && !faceReefCenter.isEnabled() && !driverRotating && !atSetpoint;
                boolean useFaceReef = faceReefCenter.isEnabled() && !autoHeading.isEnabled() && !driverRotating && !atSetpoint;

                if (useAutoHeading) {
                    autoHeading.updateTargetHeading(drivetrain.getPose());
                    
                    drivetrain.setControl(
                        headingDrive
                            .withVelocityX(xSpeed * MaxSpeed)
                            .withVelocityY(ySpeed * MaxSpeed)
                            .withTargetDirection(autoHeading.getTargetHeading())
                    );
                } else if (useFaceReef) {
                    faceReefCenter.updateTargetHeading(drivetrain.getPose());
                    
                    drivetrain.setControl(
                        headingDrive
                            .withVelocityX(xSpeed * MaxSpeed)
                            .withVelocityY(ySpeed * MaxSpeed)
                            .withTargetDirection(faceReefCenter.getTargetHeading())
                    );
                } else {
                    drivetrain.setControl(
                        drive
                            .withVelocityX(xSpeed * MaxSpeed)
                            .withVelocityY(ySpeed * MaxSpeed)
                            .withRotationalRate(rotSpeed * MaxAngularRate)
                    );
                }
            })
        );


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    /*
     * Final check for gyro initialization at Auto start.
     * If gyro hasn't been initialized, try one last time from cameras.
     * If cameras don't see anything, zero the gyro normally.
    */
    public void finalGyroCheck() {
        if (!vision.isGyroInitialized()) {
            boolean success = vision.forceSetYawFromCameras(drivetrain);
            if (!success) {
                // No cameras saw tags, zero normally
                drivetrain.seedFieldCentric();
                vision.setGyroInitialized();
            }
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
