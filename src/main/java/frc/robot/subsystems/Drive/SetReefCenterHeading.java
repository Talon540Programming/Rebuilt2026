package frc.robot.subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Vision.VisionBase;

public class SetReefCenterHeading{

    private final VisionBase vision;
    
    private boolean faceReefEnabled = false;
    private Rotation2d targetHeading = new Rotation2d();
    private Pose2d lastUpdatePose = new Pose2d();
    private static final double UPDATE_DISTANCE_THRESHOLD = 0.1;

    public SetReefCenterHeading(VisionBase vision) {
        this.vision = vision;
    }

    public void enableFaceReef() {
        faceReefEnabled = true;
        lastUpdatePose = new Pose2d();  // Reset so heading calculates immediately
        Logger.recordOutput("FaceReefCenter/Enabled", faceReefEnabled);
    }
    
    public void toggleFaceReef() {
        faceReefEnabled = !faceReefEnabled;
        if (faceReefEnabled) {
            lastUpdatePose = new Pose2d();  // Reset so heading calculates immediately
        }
        Logger.recordOutput("FaceReefCenter/Enabled", faceReefEnabled);
    }

    public void disableFaceReef() {
        faceReefEnabled = false;
        Logger.recordOutput("FaceReefCenter/Enabled", faceReefEnabled);
    }

    public boolean isEnabled() {
        return faceReefEnabled;
    }

    /**
     * Updates the target heading to face the center of the reef.
     * Calculates the angle from the robot's current position to the reef center.
     * 
     * @param currentPose The current pose of the robot
     */
    public void updateTargetHeading(Pose2d currentPose) {
        // Only recalculate if robot has moved significantly
        if (currentPose.getTranslation().getDistance(lastUpdatePose.getTranslation()) < UPDATE_DISTANCE_THRESHOLD) {
            return;  // Keep using the existing targetHeading
        }
        
        lastUpdatePose = currentPose;
        
        Pose2d reefCenter = vision.isRedAlliance() 
            ? FieldPoses.redCenterOfReef 
            : FieldPoses.blueCenterOfReef;
    
        double dx = reefCenter.getX() - currentPose.getX();
        double dy = reefCenter.getY() - currentPose.getY();
        
        targetHeading = new Rotation2d(Math.atan2(dy, dx)).plus(Rotation2d.k180deg);
    
        Logger.recordOutput("FaceReefCenter/TargetHeading", targetHeading.getDegrees());
        Logger.recordOutput("FaceReefCenter/ReefCenter", reefCenter);
    }

    public Rotation2d getTargetHeading() {
        return targetHeading;
    }

    public boolean isDriverRotating(double rotationInput) {
        return Math.abs(rotationInput) > OperatorConstants.deadband;
    }

}