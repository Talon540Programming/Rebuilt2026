package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs{
        public boolean hasTarget = false;
        public double limelightTX = 0.0;
        public double limelightTY = 0.0;
        public double limelightTA = 0.0;
        public Pose2d pose = new Pose2d();
        public double limelightTimestamp = 0.0;
        public int seenTagCount = 0;
        public boolean isRedAlliance = false;
        public double avgTagDistance = 0.0;
        public String cameraName = "";
    }

    default void updateVisionIOInputs(VisionIOInputs ioOne, VisionIOInputs ioTwo){}


    default void updateLimelightYaw(CommandSwerveDrivetrain drivetrain){}  
}
