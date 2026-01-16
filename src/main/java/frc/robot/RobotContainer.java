// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.Drive.SetHubHeading;
import frc.robot.subsystems.Intake.IntakeBase;
import frc.robot.subsystems.Intake.Pivot.PivotIOKraken;
import frc.robot.subsystems.Intake.Pivot.PivotIOSim;
import frc.robot.subsystems.Intake.Roller.RollerIOKraken;
import frc.robot.subsystems.Intake.Roller.RollerIOSim;
import frc.robot.subsystems.Shooter.ShooterBase;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOKraken;
import frc.robot.subsystems.Shooter.Flywheel.FlywheelIOSim;
import frc.robot.subsystems.Shooter.Hood.HoodIOKraken;
import frc.robot.subsystems.Shooter.Hood.HoodIOSim;
import frc.robot.subsystems.Shooter.Kickup.KickupIOKraken;
import frc.robot.subsystems.Shooter.Kickup.KickupIOSim;
import frc.robot.subsystems.Vision.VisionBase;
import frc.robot.subsystems.Vision.VisionIOLimelight;
import frc.robot.utility.ShootingCalculator;
import frc.robot.utility.Telemetry;
import edu.wpi.first.math.filter.SlewRateLimiter;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.generated.TunerConstants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    private final SetHubHeading autoHeading = new SetHubHeading(vision);
    private final PivotIOKraken pivotIO = new PivotIOKraken();
    private final RollerIOKraken rollerIOKraken = new RollerIOKraken();
    private final RollerIOSim rollerIOSim = new RollerIOSim();
    private final PivotIOSim pivotIOSim = new PivotIOSim();
    private final FlywheelIOKraken flywheelIOKraken = new FlywheelIOKraken();
    private final FlywheelIOSim flywheelIOSim = new FlywheelIOSim();
    private final HoodIOKraken hoodIOKraken = new HoodIOKraken();
    private final HoodIOSim hoodIOSim = new HoodIOSim();
    private final KickupIOKraken kickupIOKraken = new KickupIOKraken();
    private final KickupIOSim kickupIOSim = new KickupIOSim(); 
    private final IntakeBase intake;
    private final ShooterBase shooter;


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

        if (Robot.isSimulation()) {
        intake = new IntakeBase(pivotIOSim, rollerIOSim);
        shooter = new ShooterBase(flywheelIOSim, hoodIOSim, kickupIOSim);
        } 
        else {
        intake = new IntakeBase(pivotIO, rollerIOKraken);
        shooter = new ShooterBase(flywheelIOKraken, hoodIOKraken, kickupIOKraken);
        }

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
                autoHeading.disableFaceHub();
            }
            else{
            autoHeading.toggleFaceHub();
            }
        }));

        m_driverController.rightTrigger(0.2).whileTrue(
            Commands.run(() -> {
                var pose = drivetrain.getPose();
                boolean isRed = DriverStation.getAlliance()
                    .map(a -> a == Alliance.Red)
                    .orElse(false);

                var sol = ShootingCalculator.calculateSolution(pose, isRed);
                double initialVelFPS = ShootingCalculator.calculateInitialVelocityFPS(sol.distanceMeters);

                System.out.printf(
                    "[ShooterCalc] dist=%.2fm angle=%.2fdeg flywheel=%.0fRPM initialVel=%.2fft/s%n",
                    sol.distanceMeters, sol.getHoodAngleDegrees(), sol.flywheelRPM, initialVelFPS
                );
            })
        );



        // Simulation test triggers
        if (Robot.isSimulation()) {
            // Press Y to simulate collision
            m_driverController.y().whileTrue(
                Commands.runOnce(() -> simulateIntakeCollision(true))
            ).onFalse(
                Commands.runOnce(() -> simulateIntakeCollision(false))
            );
            
            // Press X to simulate game piece
            m_driverController.x().whileTrue(
                Commands.runOnce(() -> simulateGamePiece(true))
            ).onFalse(
                Commands.runOnce(() -> simulateGamePiece(false))
            );

            // A button to deploy intake, release to retract
            m_driverController.a().whileTrue(intake.deployCommand());

            // Or use left bumper for toggle
            m_driverController.leftBumper().onTrue(intake.toggleCommand());
        }
        
        headingDrive.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        headingDrive.HeadingController.setPID(6, .5, 0);

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
                
                // Determine which auto-heading mode to use (if any)
                boolean useAutoHeading = autoHeading.isEnabled() && !driverRotating;

                if (useAutoHeading) {
                    autoHeading.updateTargetHeading(drivetrain.getPose());
                    
                    drivetrain.setControl(
                        headingDrive
                            .withVelocityX(xSpeed * MaxSpeed)
                            .withVelocityY(ySpeed * MaxSpeed)
                            .withTargetDirection(autoHeading.getTargetHeading())
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
    private void simulateIntakeCollision(boolean colliding) {
        if (pivotIOSim != null) {
        pivotIOSim.simulateCollision(colliding);
        }
    }

    private void simulateGamePiece(boolean contact) {
        if (rollerIOSim != null) {
            rollerIOSim.simulateGamePieceContact(contact);
        }
    }

    public Command getIntakeHomingCommand() {
        if(!intake.isHomed()){
            return intake.homingSequence();
        }

        return Commands.none();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Command getHoodHomingCommand() {
        // Assuming you have a shooter subsystem instance
        if (!shooter.isHoodHomed()) {
            return shooter.hoodHomingSequence();
        }
        return Commands.none();
}

}
