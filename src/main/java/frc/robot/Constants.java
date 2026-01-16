// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;


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
    public static final int kDriverControllerPort = 0;
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
    public static final double kBackDistanceInches = 20.0;      // Distance from shooter exit to robot center
    public static final double kShooterHeightInches = 20.0;     // Height of shooter exit from ground
    public static final double kHeightOfHubInches = 72.0;       // Height of hub opening
    public static final double kClearanceInches = 12.0;         // Desired clearance over hub rim
    
    // Derived value
    public static final double kMaxHeightInches = (kHeightOfHubInches + kClearanceInches) - kShooterHeightInches;
    
    // Physics
    public static final double kGravityInchesPerSecSq = 386.09; // Gravity in inches/sec^2
    
    // Flywheel physical properties
    public static final double kFlywheelRadiusInches = 2.0;     // Radius of flywheel wheel (Colson)
    
    // Flywheel RPM limits
    public static final double kMinFlywheelRPM = 1000.0;        // Minimum safe RPM
    public static final double kMaxFlywheelRPM = 6000.0;        // Maximum safe RPM
    
    // Hood angle limits (same as ShooterConstants for reference)
    public static final double kHoodMinAngle = Math.PI / 8;     // 22.5 degrees
    public static final double kHoodMaxAngle = Math.PI / 2;     // 90 degrees
  }


  public static final class ClimberConstants {
    // Single climber with 2x X60 (leader/follower)
    public static final int kLeaderMotorId = 20;
    public static final int kFollowerMotorId = 21;
    
    public static final double kClimbUpDutyCycle = 0.8;
    public static final double kClimbDownDutyCycle = -0.5;
  }



}
