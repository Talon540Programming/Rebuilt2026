// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Util.Telemetry;
import frc.robot.commands.FuelManagementCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.DriveToPose;
import frc.robot.subsystems.Drive.SetReefCenterHeading;
import frc.robot.subsystems.Drive.SetReefSideHeading;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.HoodIOTalonFX;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIOTalonFX;
import frc.robot.subsystems.Indexer.IndexerSensor;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOTalonFX;
import frc.robot.subsystems.Kickup.Kickup;
import frc.robot.subsystems.Kickup.KickupIOTalonFX;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIOTalonFX;
import frc.robot.subsystems.Vision.VisionBase;
import frc.robot.subsystems.Vision.VisionIOLimelight;
import frc.robot.subsystems.Wrist.Wrist;
import frc.robot.subsystems.Wrist.WristIOTalonFX;
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

    // Shooter mechanism subsystems
    private final Shooter shooter = new Shooter(new frc.robot.subsystems.Shooter.ShooterIOSim());
    private final Hood hood = new Hood(new frc.robot.subsystems.Hood.HoodIOSim());
    private final Kickup kickup = new Kickup(new frc.robot.subsystems.Kickup.KickupIOSim());
    
    // Intake mechanism subsystems
    private final Intake intake = new Intake(new frc.robot.subsystems.Intake.IntakeIOSim());
    private final Indexer indexer = new Indexer(new frc.robot.subsystems.Indexer.IndexerIOSim());
    private final IndexerSensor indexerSensor = new IndexerSensor();
    private final Wrist wrist = new Wrist(new frc.robot.subsystems.Wrist.WristIOSim());

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
        configureSmartDashboard();
        
        autoChooser = AutoBuilder.buildAutoChooser("default auto"); //pick a default
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Configures SmartDashboard outputs for telemetry.
     */
    private void configureSmartDashboard() {
        // Shooter telemetry - updated in periodic
        SmartDashboard.putNumber("Shooter/CurrentBottomVelocityRPS", 0);
        SmartDashboard.putNumber("Shooter/CurrentTopVelocityRPS", 0);
        SmartDashboard.putNumber("Shooter/TargetVelocityRPS", 0);
        SmartDashboard.putBoolean("Shooter/AtTargetVelocity", false);
        
        // Hood telemetry
        SmartDashboard.putNumber("Hood/CurrentAngleDegrees", 0);
        SmartDashboard.putNumber("Hood/TargetAngleDegrees", 0);
        SmartDashboard.putBoolean("Hood/AtTargetPosition", false);
        
        // Shooter position reference
        double[] shooterOffset = frc.robot.Util.TrajectoryCalculator.getShooterOffsetFromLimelight2();
        SmartDashboard.putNumber("Shooter/OffsetLeftFromLL2_Inches", shooterOffset[0]);
        SmartDashboard.putNumber("Shooter/OffsetForwardFromLL2_Inches", shooterOffset[1]);
        SmartDashboard.putNumber("Shooter/OffsetUpFromLL2_Inches", shooterOffset[2]);
        SmartDashboard.putNumber("Shooter/HeightAboveGround_Inches", 
            frc.robot.Util.TrajectoryCalculator.getShooterHeight());
        
        // Trajectory calculation info
        SmartDashboard.putNumber("Shooter/DistanceToTargetInches", 0);
        
        // Fuel management
        SmartDashboard.putNumber("Indexer/FuelCount", 0);
        SmartDashboard.putBoolean("Indexer/HasFuel", false);
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

        // ========== INTAKE AND INDEXER CONTROLS ==========
        
        // Right Trigger - Full intake sequence with fuel counting
        m_driverController.rightTrigger().whileTrue(
            FuelManagementCommands.intakeWithCounting(intake, indexer, wrist, indexerSensor)
        ).onFalse(
            IntakeCommands.stopIntake(intake, indexer, wrist)
        );
        
        // A Button - Deploy wrist
        m_driverController.a().onTrue(
            IntakeCommands.deployWrist(wrist)
        );
        
        // Back Button - Retract wrist (stow)
        m_driverController.back().onTrue(
            IntakeCommands.retractWrist(wrist)
        );
        
        
        // ========== SHOOTER CONTROLS ==========
        
        // X Button - Auto-aim and shoot sequence with proper velocity waiting
        // Order: 1) Calculate trajectory, 2) Set hood angle and spin up flywheel, 3) Wait for ready, 4) Start kickup
        m_driverController.x().whileTrue(
            Commands.sequence(
                // Step 1: Calculate trajectory and command shooter/hood (runs once)
                Commands.runOnce(() -> {
                    var robotPose = drivetrain.getPose();
                    var targetPose = frc.robot.Util.TrajectoryCalculator.getTargetPose(vision.isRedAlliance());
                    
                    frc.robot.Util.TrajectoryCalculator calc = new frc.robot.Util.TrajectoryCalculator();
                    
                    if (calc.calculateTrajectory(robotPose, targetPose)) {
                        // Set hood angle
                        double hoodAngle = calc.getLaunchAngleRadians();
                        hood.setAngleRadians(hoodAngle);
                        
                        // Set shooter velocity
                        double launchVelocity = calc.getLaunchVelocityFeetPerSecond();
                        double wheelDiameter = 4.0; // TODO: Measure actual wheel diameter
                        double flywheelRPS = frc.robot.Util.TrajectoryCalculator.velocityToFlywheelRPS(
                            launchVelocity, 
                            wheelDiameter
                        );
                        shooter.setVelocity(flywheelRPS);
                        
                        // Log trajectory data for SmartDashboard
                        SmartDashboard.putNumber("Shooter/TargetVelocityRPS", flywheelRPS);
                        SmartDashboard.putNumber("Shooter/TargetAngleDegrees", Math.toDegrees(hoodAngle));
                        SmartDashboard.putNumber("Shooter/DistanceToTargetInches", calc.getHorizontalDistanceInches());
                    }
                }, shooter, hood),
                
                // Step 2: Keep shooter and hood active while waiting for them to reach target
                Commands.run(() -> {
                    // Continuously maintain shooter and hood commands
                    // This keeps the subsystems "required" so they can't be interrupted
                    
                    boolean shooterReady = shooter.isAtTargetVelocity();
                    boolean hoodReady = hood.isAtTargetPosition();
                    
                    // Update SmartDashboard with ready status
                    SmartDashboard.putBoolean("Shooter/ShooterAtVelocity", shooterReady);
                    SmartDashboard.putBoolean("Shooter/HoodAtPosition", hoodReady);
                }, shooter, hood).until(() -> 
                    shooter.isAtTargetVelocity() && hood.isAtTargetPosition()
                ),
                
                // Step 3: Both are ready - now start feeding fuel
                Commands.parallel(
                    Commands.run(() -> indexer.feed(), indexer),
                    Commands.run(() -> kickup.feed(), kickup)
                )
            )
        ).onFalse(
            // Stop everything when button released
            Commands.parallel(
                Commands.runOnce(() -> shooter.stop(), shooter),
                Commands.runOnce(() -> kickup.stop(), kickup),
                Commands.runOnce(() -> indexer.stop(), indexer)
            )
        );
        
        // Y Button - Eject fuel (reverse intake)
        m_driverController.y().whileTrue(
            IntakeCommands.ejectFuel(intake, indexer)
        ).onFalse(
            Commands.parallel(
                Commands.runOnce(() -> intake.stop(), intake),
                Commands.runOnce(() -> indexer.stop(), indexer)
            )
        );
        
        // POV Left - Reset fuel count
        m_driverController.povLeft().onTrue(
            FuelManagementCommands.resetFuelCount(indexerSensor)
        );

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

    /**
     * Periodic method to update SmartDashboard values.
     * Call this from Robot.java robotPeriodic() or teleopPeriodic().
     */
    public void updateTelemetry() {
        // Update shooter current velocities
        SmartDashboard.putNumber("Shooter/CurrentBottomVelocityRPS", shooter.getBottomVelocity());
        SmartDashboard.putNumber("Shooter/CurrentTopVelocityRPS", shooter.getTopVelocity());
        SmartDashboard.putNumber("Shooter/TargetVelocityRPS", shooter.getTargetVelocity());
        SmartDashboard.putBoolean("Shooter/AtTargetVelocity", shooter.isAtTargetVelocity());
        
        // Update hood current angle
        SmartDashboard.putNumber("Hood/CurrentAngleDegrees", hood.getAngleDegrees());
        SmartDashboard.putNumber("Hood/TargetAngleDegrees", Math.toDegrees(hood.getTargetPosition() * 2 * Math.PI));
        SmartDashboard.putBoolean("Hood/AtTargetPosition", hood.isAtTargetPosition());
        
        // Update fuel count
        SmartDashboard.putNumber("Indexer/FuelCount", indexerSensor.getFuelCount());
        SmartDashboard.putBoolean("Indexer/HasFuel", indexerSensor.hasFuel());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}