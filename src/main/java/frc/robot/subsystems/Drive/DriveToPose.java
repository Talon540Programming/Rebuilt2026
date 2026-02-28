
// Reduntant drive to pose code incase needed its built out
package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;



public class DriveToPose{

        private final CommandSwerveDrivetrain drivetrain;

        // Left & Right from driver station perspective 
        public enum Side{
            AlignLeft,
            AlignRight,
            ClimbLeft,
            ClimbRight
        }
    
        public DriveToPose(CommandSwerveDrivetrain drivetrain){
            this.drivetrain = drivetrain;
        }
        
    
        private Pose2d calculateTowerPose(Side side, Pose2d selectedSide){
            double x = selectedSide.getX();
            double y = selectedSide.getY();
            double rot = selectedSide.getRotation().getRadians();
            //tune value with more testing, rotational offset 
            rot += Math.toRadians(0);
    
            switch (side) {
                case AlignLeft:
                    x -= DriveConstants.alignTowerLateralOffset * Math.sin(rot);
                    y += DriveConstants.alignTowerLateralOffset * Math.cos(rot);
                    break;
                case AlignRight:
                    x += DriveConstants.alignTowerLateralOffset * Math.sin(rot);
                    y -= DriveConstants.alignTowerLateralOffset * Math.cos(rot);
                    break;
                case ClimbLeft:
                    x -= DriveConstants.climbTowerLateralOffset * Math.sin(rot);
                    y += DriveConstants.climbTowerLateralOffset * Math.cos(rot);
                    break;
                case ClimbRight:
                    x += DriveConstants.climbTowerLateralOffset * Math.sin(rot);
                    y -= DriveConstants.climbTowerLateralOffset * Math.cos(rot);
                    break;
            }
    
            x += (DriveConstants.towerDistanceOffset) * Math.cos(rot);
            y += (DriveConstants.towerDistanceOffset) * Math.sin(rot);
    
             return new Pose2d(x, y, new Rotation2d(rot));
        }

    
    private Command getPathFromWaypoint(Pose2d waypoint){
        // Calculate approach direction (from robot toward target)
        var approachDirection = waypoint.getTranslation().minus(drivetrain.getPose().getTranslation()).getAngle();
        
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(drivetrain.getPose().getTranslation(), getPathVelocityHeading(drivetrain.getFieldVelocity(), waypoint)),
            new Pose2d(waypoint.getTranslation(), approachDirection)  // Approach FROM robot's direction, not final heading
        );
        
        if(waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.05){
            return Commands.none();
        }
    
        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            Constants.PathPlannerConstants.defaultConstraints,
            new IdealStartingState(Math.max(drivetrain.getVelocityMagnitude().in(MetersPerSecond), 1.0), drivetrain.getHeading()), 
            new GoalEndState(0.0, waypoint.getRotation()));  // Final ROBOT heading stays here
        
        path.preventFlipping = true;
    
        return AutoBuilder.followPath(path);
    }
        
    private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target){
        // Calculate direction toward target
        var diff = target.getTranslation().minus(drivetrain.getPose().getTranslation());
        
        // If very close to target, use direction toward target (or current heading if basically on top of it)
        if (diff.getNorm() < 0.01) {
            return drivetrain.getPose().getRotation();
        }
        
        // If robot is moving slowly, point path toward target
        if (drivetrain.getVelocityMagnitude().in(MetersPerSecond) < 0.25) {
            return diff.getAngle();
        }
        
        // If robot is moving, use velocity direction to create smooth path
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

     
    public boolean isAtSetpoint(double toleranceMeters, Side side, Pose2d selectedSide) {
        Pose2d currentPose = drivetrain.getPose();
        
        // Check distance to nearest tower
        Pose2d selectedTower = calculateTowerPose(side, selectedSide);
        double towerDistance = currentPose.getTranslation().getDistance(selectedTower.getTranslation());
        
        return towerDistance < toleranceMeters;
    }
    

    public boolean isAtTargetPose(Pose2d target, double toleranceMeters) {
        return drivetrain.getPose().getTranslation().getDistance(target.getTranslation()) < toleranceMeters;
    }

    private Pose2d getCorrectPose(Side side){
        Pose2d correctPose = new Pose2d();
        var alliance = DriverStation.getAlliance();
        switch (side) {
            case AlignLeft:
                correctPose = (alliance.get() == DriverStation.Alliance.Red)? 
                    Constants.FieldPoses.alignRedTowerLeft: 
                    Constants.FieldPoses.alignBlueTowerLeft;
                break;
            case AlignRight:
                correctPose = (alliance.get() == DriverStation.Alliance.Red)? 
                    Constants.FieldPoses.alignRedTowerRight: 
                    Constants.FieldPoses.alignBlueTowerRight;
                break;
            case ClimbLeft:
                correctPose = (alliance.get() == DriverStation.Alliance.Red)? 
                    Constants.FieldPoses.climbRedTowerLeft: 
                    Constants.FieldPoses.climbBlueTowerLeft;
                    break;
            case ClimbRight:
                correctPose = (alliance.get() == DriverStation.Alliance.Red)? 
                    Constants.FieldPoses.climbRedTowerRight: 
                    Constants.FieldPoses.climbBlueTowerRight;
                    break;
        }

        return correctPose;
    }
        
    public Command createtowerPathCommand(Side side){
        return Commands.defer(()-> {
            Pose2d selectedSide = getCorrectPose(side);
            Pose2d towerSideSelected = calculateTowerPose(side, selectedSide);
            
            if(isAtTargetPose(towerSideSelected, 0.05)){
                return Commands.none();
            }

            return getPathFromWaypoint(towerSideSelected);
        }, Set.of(drivetrain));
    }

}
