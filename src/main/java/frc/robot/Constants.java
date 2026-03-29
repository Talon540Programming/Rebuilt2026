// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.utility.LoggedTunableNumber;
import edu.wpi.first.math.util.Units;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

   public static final boolean TUNING_MODE = true;

   private static RobotType robotType = RobotType.SIMBOT;

    // Add these enums
    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public enum RobotType {
        SIMBOT,
        COMPBOT
    }

    public static RobotType getRobot() {
        if (RobotBase.isReal()) {
            return RobotType.COMPBOT;
        }
        return robotType;  // Use the configured type for simulation (SIMBOT or REPLAY)
    }

    public static Mode getMode() {
        return switch (robotType) {
            case COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
            case SIMBOT -> Mode.SIM;
        };
    }

  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
    public static final double deadband = 0.1;
  }

  public static final int PDHCanId = 1;

  public static final class PathPlannerConstants {

    public static final PathConstraints testingConstraints = new PathConstraints(
        Units.feetToMeters(1.5), 2.0,             
        Units.degreesToRadians(50), Units.degreesToRadians(300));

    public static final PathConstraints slowConstraints = new PathConstraints(
        Units.feetToMeters(3.5), 4.0,             
        Units.degreesToRadians(100), Units.degreesToRadians(720));

    public static final PathConstraints defaultConstraints = new PathConstraints(
        Units.feetToMeters(8), 4.0,
        Units.degreesToRadians(200), Units.degreesToRadians(720));

    public static final PathConstraints fastConstraints = new PathConstraints(
      Units.feetToMeters(14), 8.0,
        Units.degreesToRadians(600), Units.degreesToRadians(1000));
  }

  public static final class FieldPoses {

    public static final double[] fieldSize = {17.55, 8.05};
    public static final double fieldLengthMeters = 17.55;
    // Wall thickness is 0.051
    // Hub positions
    public static final Pose2d blueHub = new Pose2d(4.5, 4.0, new Rotation2d());
    public static final Pose2d redHub = new Pose2d(12, 4.0, new Rotation2d()); 
    public static final Pose2d alignRedTowerLeft = new Pose2d(0.0, 0.0, new Rotation2d()); 
    public static final Pose2d alignRedTowerRight = new Pose2d(0.0, 0.0, new Rotation2d()); 
    public static final Pose2d alignBlueTowerLeft = new Pose2d(0.0, 0.0, new Rotation2d(0)); 
    public static final Pose2d alignBlueTowerRight = new Pose2d(1.282, 2.832, new Rotation2d(-Math.PI/2)); 
    public static final Pose2d climbRedTowerLeft = new Pose2d(0.0, 0.0, new Rotation2d()); 
    public static final Pose2d climbRedTowerRight = new Pose2d(0.0, 0.0, new Rotation2d()); 
    public static final Pose2d climbBlueTowerLeft = new Pose2d(0, 0, new Rotation2d()); 
    public static final Pose2d climbBlueTowerRight = new Pose2d(1.000, 2.846, new Rotation2d(-Math.PI/2)); 
    public static final Pose2d test = new Pose2d(0, 0, new Rotation2d()); 
  }

   public static final class ShootingConstants {
    // Physical measurements (in inches for trajectory calculation)
    public static final double backDistanceInches = 10.0;      
    public static final double shooterHeightInches = 19;     // Height of shooter exit from ground
    public static final double heightOfHubInches = 72.0;       // Height of hub opening
    public static final double clearanceInches = 33.0;         // Desired clearance over hub rim
    
    // Derived value
    public static final double maxHeightInches = (heightOfHubInches + clearanceInches) - shooterHeightInches;
    
    // Physics
    public static final double gravityInchesPerSecSq = 386.09; // Gravity in inches/sec^2
    
    // Flywheel physical properties
    public static final double flywheelRadiusInches = 2.0;     // Radius of flywheel wheel (Colson)
    
    // Flywheel RPM limits
    public static final double minFlywheelRPM = 100.0;        // Minimum safe RPM
    public static final double maxFlywheelRPM = 6000.0;        // Maximum safe RPM
    
    // Hood angle limits (same as ShooterConstants for reference)
    public static final double hoodMinAngle = Math.toRadians(18);     // 18 degrees
    public static final double hoodMaxAngle = Math.toRadians(55);     // 54 degrees

    public static final LoggedTunableNumber compensationFactorX = 
        new LoggedTunableNumber("Shooting/CompensationFactorX", 2.5);
    public static final LoggedTunableNumber compensationFactorY = 
        new LoggedTunableNumber("Shooting/CompensationFactorY", 2.5);

    // Shooter position offset from robot center (in inches)
    // Positive X = forward, Positive Y = left
    public static final LoggedTunableNumber shooterOffsetXInches = 
        new LoggedTunableNumber("Shooting/ShooterOffsetXInches", 6.3);
    public static final LoggedTunableNumber shooterOffsetYInches = 
        new LoggedTunableNumber("Shooting/ShooterOffsetYInches", 4.598);
        // Hood auto-retraction near trench
    public static final double hoodRetractionDistanceMeters = 1.5;
  }

  public static final class PassingConstants {
    // Distance from trench (hub X) to landing zone (meters)
    public static final double trenchToLandingDistanceMeters = 2.0;
    
    // Trench clearance requirements (inches)
    public static final double trenchHeightInches = 40.5;
    
    // Tunable margin above trench (inches)
    public static final LoggedTunableNumber trenchClearanceMarginInches = 
        new LoggedTunableNumber("Passing/TrenchClearanceMarginInches", 6.0);

    // Fixed angle for derived passing method (degrees)
    public static final double fixedPassingAngleDegrees = 40.0;
  }

  public static final class HeadingPID {
    public static LoggedTunableNumber headingP =
        new LoggedTunableNumber("AutoSetHeading/HeadingP");
    public static LoggedTunableNumber headingI =
        new LoggedTunableNumber("AutoSetHeading/HeadingI");
    public static LoggedTunableNumber headingD =
        new LoggedTunableNumber("AutoSetHeading/HeadingD");
    public static LoggedTunableNumber shooterThetaOffset =
        new LoggedTunableNumber("Shooter/ShooterThetaOffset", Math.toRadians(17.443637));
    public static LoggedTunableNumber headingToleranceRadians = 
        new LoggedTunableNumber("AutoSetHeading/HeadingToleranceRadians", Math.toRadians(5));
  }

  static {
    switch (getRobot()) {
      case SIMBOT ->{
        HeadingPID.headingP.initDefault(20.0);
        HeadingPID.headingI.initDefault(0.0);
        HeadingPID.headingD.initDefault(0.5);

      }
      case COMPBOT ->{
        HeadingPID.headingP.initDefault(5); //TODO Tuned for proto
        HeadingPID.headingI.initDefault(0.0); //TODO Tuned for proto
        HeadingPID.headingD.initDefault(1); //TODO Tuned for proto
      }
    }
  }

  public static final class RobotDimensions {
    // Robot dimensions WITH bumpers (in meters)
    public static final double robotWidthMeters = Units.inchesToMeters(29.0);  // left to right (Y axis)
    public static final double robotLengthMeters = Units.inchesToMeters(29.0); // front to back (X axis)
    public static final double bumperHeightMeters = Units.inchesToMeters(5.5); // floor to top of bumpers
  }

  public static final class EmergencyModeConstants {
    // Emergency shooting preset (hub shot)
    public static final double shootingRPM = 500;
    public static final double shootingHoodAngleDegrees = 18;
    
    // Emergency passing preset (lob shot) - TODO: Update with real values
    public static final double passingRPM = 1700;
    public static final double passingHoodAngleDegrees = 18;
  }

}
