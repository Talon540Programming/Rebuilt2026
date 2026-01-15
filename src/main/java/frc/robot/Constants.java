// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double deadband = 0.1;
  }

  public static final String kLimelightOne = "limelight-one";
  public static final String kLimelightTwo = "limelight-two";

  public static final int PDHCanId = 1;

  public static enum Reef {
    left,
    right,
    forward;
  }


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
  
    
    public static LoggedNetworkNumber reefLateralOffset = 
        new LoggedNetworkNumber("ReefAlign/LateralOffset", 0.2);
    public static LoggedNetworkNumber reefDistanceOffset = 
        new LoggedNetworkNumber("ReefAlign/ReefDistanceOffset", -0.05);
        public static LoggedNetworkNumber stationLateralOffset = 
        new LoggedNetworkNumber("ReefAlign/LateralOffset", 0.2);
    public static LoggedNetworkNumber stationDistanceOffset = 
        new LoggedNetworkNumber("ReefAlign/ReefDistanceOffset", 0.0);
        
    public static final double bumperWidth = -0.1;

  }

  public static final class IntakeConstants {
  // existing IDs...
  public static final int kPivotMotorId = 13;
  public static final int kRollerMotorId = 14;

  // ===================== Pivot Motion Magic =====================
  /** Rotor rotations per 1 mechanism rotation (gear ratio). REQUIRED for correct units. */
  public static final double kPivotSensorToMechanismRatio = 0; /* TODO */;

  /** Mechanism rotations for stowed/deployed. Pick either rotations OR compute from degrees. */
  public static final double kPivotStowedPosRot = 0; /* TODO */
  public static final double kPivotDeployedPosRot = 0; /* TODO */

  public static final double kDeployDutyCycle = 0.5;
  public static final double kRollerIntakeDutyCycle = 0.8;
  public static final double kRollerEjectDutyCycle = -0.5;
  public static final double kCollisionCurrentThreshold = 25;

  /** How close is “at goal” */
  public static final double kPivotPosToleranceRot = 0; /* TODO e.g. 0.01 */
  public static final double kPivotVelToleranceRotPerSec = 0; /* TODO e.g. 0.05 */

  // Motion Magic constraints (mechanism units if SensorToMechanismRatio is set)
  public static final double kPivotMMCruiseVelRotPerSec = 0; /* TODO */
  public static final double kPivotMMAccelRotPerSec2 = 0; /* TODO */
  public static final double kPivotMMJerkRotPerSec3 = 0; /* TODO (optional) */

  // Slot0 PID (tune on real robot)
  public static final double kPivotkP = 0; /* TODO */
  public static final double kPivotkI = 0; 
  public static final double kPivotkD = 0; /* TODO */

  // Optional gravity/feedforward support (if you use it)
  public static final double kPivotkG = 0; /* TODO maybe 0 to start */

  // ===================== Crash / stall detection =====================
  public static final double kPivotCrashCurrentAmps = 0; // you already have collision threshold
  public static final double kPivotCrashMinErrorRot = 0; /* TODO e.g. 0.05 */;
  public static final double kPivotCrashMaxVelRotPerSec = 0; /* TODO e.g. 0.1 */;
  public static final double kPivotCrashDebounceSecs = 0; /* TODO e.g. 0.15 */;

  public static final double kPivotAllowedErrorRot = 0;
  public static final double kPivotAllowedVelRotPerSec = 0;
  public static final double kPivotCrashIgnoreAfterGoalChangeSecs = 0;

}

  public static final class IndexConstants {
    public static final int kIndexMotorId = 15;
    public static final int kCANRangeId = 22;
    
    public static final double kIndexDutyCycle = 0.5;      // Normal indexing speed
    public static final double kReverseDutyCycle = -0.3;   // Reverse/eject speed
    public static final double kFeedDutyCycle = 0.8;       // Fast feed during shooting
    
    // CANRange detection threshold in meters - tune on real robot
    public static final double kGamePieceDetectionThreshold = 0.15;
  }

   public static final class ShooterConstants {
    // Flywheel motors (2x X60 on same mechanism)
    public static final int kFlywheelMotor1Id = 17;
    public static final int kFlywheelMotor2Id = 18;
    
    // Hood motor (X44 rack and pinion)
    public static final int kHoodMotorId = 19;
    
    // Hood angle limits (radians)
    public static final double kHoodMinAngle = Math.PI / 8;  // 22.5 degrees
    public static final double kHoodMaxAngle = Math.PI / 2;  // 90 degrees
    
    // Flywheel PID constants - tune on real robot
    public static final double kFlywheelkP = 0.1;
    public static final double kFlywheelkI = 0.0;
    public static final double kFlywheelkD = 0.0;
    public static final double kFlywheelkV = 0.12; // Feedforward
    
    // Hood PID constants - tune on real robot
    public static final double kHoodkP = 1.0;
    public static final double kHoodkI = 0.0;
    public static final double kHoodkD = 0.0;
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

   public static final class KickupConstants {
    public static final int kKickupMotorId = 16;
    
    public static final double kFeedDutyCycle = 0.8;
    public static final double kReverseDutyCycle = -0.3;
  }

}
