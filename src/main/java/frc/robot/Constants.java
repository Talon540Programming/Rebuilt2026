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
        return robotType;
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
    // Wall thickness is 0.051
    // Hub positions for 2026 REBUILT game - TODO: Update with actual field coordinates
    
    public static final Pose2d blueHub = new Pose2d(4.5, 4.0, new Rotation2d()); // Placeholder
    public static final Pose2d redHub = new Pose2d(12, 4.0, new Rotation2d());  // Placeholder

  }

   public static final class ShootingConstants {
    // Physical measurements (in inches for trajectory calculation)
    public static final double backDistanceInches = 20.0;      // Distance from shooter exit to robot center
    public static final double shooterHeightInches = 20.0;     // Height of shooter exit from ground
    public static final double heightOfHubInches = 72.0;       // Height of hub opening
    public static final double clearanceInches = 12.0;         // Desired clearance over hub rim
    
    // Derived value
    public static final double maxHeightInches = (heightOfHubInches + clearanceInches) - shooterHeightInches;
    
    // Physics
    public static final double gravityInchesPerSecSq = 386.09; // Gravity in inches/sec^2
    
    // Flywheel physical properties
    public static final double flywheelRadiusInches = 2.0;     // Radius of flywheel wheel (Colson)
    
    // Flywheel RPM limits
    public static final double minFlywheelRPM = 1000.0;        // Minimum safe RPM
    public static final double maxFlywheelRPM = 6000.0;        // Maximum safe RPM
    
    // Hood angle limits (same as ShooterConstants for reference)
    public static final double hoodMinAngle = Math.PI / 8;     // 22.5 degrees
    public static final double hoodMaxAngle = Math.PI / 2;     // 90 degrees
  }

  public static final class HeadingPID{
    public static LoggedTunableNumber headingP =
      new LoggedTunableNumber("AutoSetHeading/HeadingP");
    public static LoggedTunableNumber headingI =
      new LoggedTunableNumber("AutoSetHeading/HeadingI");
    public static LoggedTunableNumber headingD =
      new LoggedTunableNumber("AutoSetHeading/HeadingD"); 
  }

  static {
    switch (getRobot()) {
      case SIMBOT ->{
        HeadingPID.headingP.initDefault(25.0);
        HeadingPID.headingI.initDefault(0.0);
        HeadingPID.headingD.initDefault(0.0);

      }
      case COMPBOT ->{
        HeadingPID.headingP.initDefault(0.0); //TODO
        HeadingPID.headingI.initDefault(0.0); //TODO
        HeadingPID.headingD.initDefault(0.0); //TODO
      }
    }
  }

}
