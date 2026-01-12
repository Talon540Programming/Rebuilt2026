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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.FieldPoses;
import frc.robot.subsystems.Vision.VisionBase;



public class DriveToPose{

        private final CommandSwerveDrivetrain drivetrain;
        private VisionBase vision;
        private Pose2d nearestReefSide = new Pose2d();
        private Pose2d nearestStation = new Pose2d();
        
        public enum Side{
            Left,
            Middle,
            Right
        }
    
        public DriveToPose(CommandSwerveDrivetrain drivetrain,VisionBase vision){
            this.drivetrain = drivetrain;
            this.vision = vision;
        }
        
        public Pose2d calculateNearestReefSide(){
            if(vision.isRedAlliance()){
                return drivetrain.getPose().nearest(FieldPoses.redReefPoses);
            }
            else{
                return drivetrain.getPose().nearest(FieldPoses.blueReefPoses);
            }
        }

        public Pose2d calculateNearestStation(){
            if(vision.isRedAlliance()){
                return drivetrain.getPose().nearest(FieldPoses.redStationPoses);
            }
            else{
                return drivetrain.getPose().nearest(FieldPoses.blueStationPoses);
            }
        }
    
        private Pose2d calculateReefPath(Side side, Pose2d nearestSide){
            double x = nearestSide.getX();
            double y = nearestSide.getY();
            double rot = nearestSide.getRotation().getRadians();
            //tune value with more testing, rotational offset 
            rot += Math.toRadians(0);
    
            switch (side) {
                case Left:
                    x -= FieldPoses.reefLateralOffset.get() * Math.sin(rot);
                    y += FieldPoses.reefLateralOffset.get() * Math.cos(rot);
                break;
                case Right:
                    x += FieldPoses.reefLateralOffset.get() * Math.sin(rot);
                    y -= FieldPoses.reefLateralOffset.get() * Math.cos(rot);
                break;
                case Middle:
                   default:
    
                break;
            }
    
            x += (FieldPoses.reefDistanceOffset.get() + FieldPoses.bumperWidth) * Math.cos(rot);
            y += (FieldPoses.reefDistanceOffset.get() + FieldPoses.bumperWidth) * Math.sin(rot);
    
             return new Pose2d(x, y, new Rotation2d(rot));
        }  

    private Pose2d calculateStationPath(Pose2d station){
        double x = station.getX();
        double y = station.getY();
        double rot = station.getRotation().getRadians();
        //tune value with more testing, rotational offset 

        x += (FieldPoses.stationDistanceOffset.get() + FieldPoses.bumperWidth) * Math.cos(rot);
        y += (FieldPoses.stationDistanceOffset.get() + FieldPoses.bumperWidth) * Math.sin(rot);

            return new Pose2d(x, y, new Rotation2d(rot));
    }  

    public boolean haveReefConditionsChanged(){
        Pose2d nearSide = calculateNearestReefSide();
        
        return !nearSide.equals(nearestReefSide);
    }

    public boolean haveStationConditionsChanged(){
        Pose2d station = calculateNearestStation();
        
        return !station.equals(nearestStation);
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

    public boolean isAtSetpoint(double toleranceMeters) {
        Pose2d currentPose = drivetrain.getPose();
        
        // Check distance to nearest reef
        Pose2d nearestReef = calculateNearestReefSide();
        double reefDistance = currentPose.getTranslation().getDistance(nearestReef.getTranslation());
        
        // Check distance to nearest station
        Pose2d nearestStation = calculateNearestStation();
        double stationDistance = currentPose.getTranslation().getDistance(nearestStation.getTranslation());
        
        return reefDistance < toleranceMeters || stationDistance < toleranceMeters;
    }

    public boolean isAtTargetPose(Pose2d target, double toleranceMeters) {
        return drivetrain.getPose().getTranslation().getDistance(target.getTranslation()) < toleranceMeters;
    }
        
    public Command createReefPathCommand(Side side){
        return Commands.defer(()-> {
            nearestReefSide = calculateNearestReefSide();
            Pose2d align = calculateReefPath(side, nearestReefSide);
            
            if(isAtTargetPose(align, 0.05)){
                return Commands.none();
            }

            return getPathFromWaypoint(align);
        }, Set.of(drivetrain));
    }

    public Command createStationPathCommand(){
        return Commands.defer(()-> {
            nearestStation = calculateNearestStation();
            Pose2d align = calculateStationPath(nearestStation);

            if(isAtTargetPose(align, 0.08)){
                return Commands.none();
            }

            return getPathFromWaypoint(align);
        }, Set.of(drivetrain));
    }
}
