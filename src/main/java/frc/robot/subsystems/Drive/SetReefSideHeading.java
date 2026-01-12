package frc.robot.subsystems.Drive;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldPoses;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Vision.VisionBase;


public class SetReefSideHeading{

    private final VisionBase vision;

    private boolean autoHeadingEnabled = false;
    private Rotation2d targetHeading = new Rotation2d();
    private Pose2d nearestReefFace = new Pose2d();

    public SetReefSideHeading(VisionBase vision){
        this.vision = vision;
    }

    public void toggleAutoHeading(){
        autoHeadingEnabled = !autoHeadingEnabled;
        Logger.recordOutput("AutoHeading/Enabled", autoHeadingEnabled);
    }

    public void enableAutoHeading(){
        autoHeadingEnabled = true;
        Logger.recordOutput("AutoHeading/Enabled", autoHeadingEnabled);
    }

    public void disableAutoHeading(){
        autoHeadingEnabled = false;
        Logger.recordOutput("AutoHeading/Enabled", autoHeadingEnabled);
    }

    public boolean isEnabled(){
        return autoHeadingEnabled;
    }

    public void updateTargetHeading(Pose2d currentPose){
        nearestReefFace = currentPose.nearest(
            vision.isRedAlliance() ? FieldPoses.redReefPoses : FieldPoses.blueReefPoses
        );

        targetHeading = nearestReefFace.getRotation().plus(Rotation2d.k180deg);

        Logger.recordOutput("AutoHeading/TargetHeading", targetHeading.getDegrees());
        Logger.recordOutput("AutoHeading/NearestReefFace", nearestReefFace);

    }

    public Rotation2d getTargetHeading() {
        return targetHeading;
    }

    public Pose2d getNearestReefFace() {
        return nearestReefFace;
    }

    public boolean isDriverRotating(double rotationInput) {
        return Math.abs(rotationInput) > OperatorConstants.deadband;
    }
}
