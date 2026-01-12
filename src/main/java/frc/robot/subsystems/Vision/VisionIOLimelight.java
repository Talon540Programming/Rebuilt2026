package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Util.LimelightHelpers;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class VisionIOLimelight implements VisionIO {
    private final String limelightOne = Constants.kLimelightOne;
    private final String limelightTwo = Constants.kLimelightTwo;

    public VisionIOLimelight(){
        //sets mode for Metatag 2 and mode 0 uses robot gyro for orentation 
        LimelightHelpers.SetIMUMode(limelightOne, 0);
        LimelightHelpers.SetIMUMode(limelightTwo, 0);


    }

    public void updateVisionIOInputs(VisionIOInputs inputOne, VisionIOInputs inputTwo) {
        updateSingleCamera(limelightOne, inputOne);
        updateSingleCamera(limelightTwo, inputTwo);
    }
    
    private void updateSingleCamera(String cameraName, VisionIOInputs input) {
        input.cameraName = cameraName;
        input.hasTarget = LimelightHelpers.getTV(cameraName);
        input.limelightTX = LimelightHelpers.getTX(cameraName);
        input.limelightTY = LimelightHelpers.getTY(cameraName);
        input.limelightTA = LimelightHelpers.getTA(cameraName);
    
        var alliance = DriverStation.getAlliance();
        if (!alliance.isEmpty() && alliance.get() == DriverStation.Alliance.Red) {
            input.isRedAlliance = true;
        } else if (!alliance.isEmpty() && alliance.get() == DriverStation.Alliance.Blue) {
            input.isRedAlliance = false;
        }
    
        LimelightHelpers.PoseEstimate metaTag2Pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    
        if (metaTag2Pose != null) {
            input.pose = metaTag2Pose.pose;
            input.limelightTimestamp = metaTag2Pose.timestampSeconds;
            input.seenTagCount = metaTag2Pose.tagCount;
            input.avgTagDistance = metaTag2Pose.avgTagDist;
        } else {
            input.seenTagCount = 0;
        }
    }
    @Override
    public void updateLimelightYaw(CommandSwerveDrivetrain drivetrain) {
        // The pose rotation should be in field coordinates after seedFieldCentric
        // But we need to verify this is what the Limelight expects
        double yaw = drivetrain.getPose().getRotation().getDegrees();
        
        LimelightHelpers.SetRobotOrientation(limelightOne, yaw, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(limelightTwo, yaw, 0, 0, 0, 0, 0);
    }
}
