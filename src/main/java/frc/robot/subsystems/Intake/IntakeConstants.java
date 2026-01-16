package frc.robot.subsystems.Intake;

import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class IntakeConstants {
    // Shared constants (same for sim and real)
    public static final int kPivotMotorId = 13;
    public static final int kRollerMotorId = 14;

    public static final double kPivotHomingDutyCycle = -0.15;
    public static final double kPivotHomingVelThreshold = 0.05;
    public static final double kPivotHomingCurrentThreshold = 5.0;

    public static final double kDeployDutyCycle = 0.5;
    public static final double kRollerIntakeDutyCycle = 0.8;
    public static final double kRollerEjectDutyCycle = -0.5;

    // ===================== Pivot Motion Magic =====================
    public static final LoggedTunableNumber pivotSensorToMechanismRatio = 
        new LoggedTunableNumber("Intake/Pivot/SensorToMechanismRatio");
    public static final LoggedTunableNumber pivotStowedPosRot = 
        new LoggedTunableNumber("Intake/Pivot/StowedPosRot");
    public static final LoggedTunableNumber pivotDeployedPosRot = 
        new LoggedTunableNumber("Intake/Pivot/DeployedPosRot");
    public static final LoggedTunableNumber pivotPosToleranceRot = 
        new LoggedTunableNumber("Intake/Pivot/PosToleranceRot");
    public static final LoggedTunableNumber pivotVelToleranceRotPerSec = 
        new LoggedTunableNumber("Intake/Pivot/VelToleranceRotPerSec");

    // Motion Magic constraints
    public static final LoggedTunableNumber pivotMMCruiseVelRotPerSec = 
        new LoggedTunableNumber("Intake/Pivot/MMCruiseVelRotPerSec");
    public static final LoggedTunableNumber pivotMMAccelRotPerSec2 = 
        new LoggedTunableNumber("Intake/Pivot/MMAccelRotPerSec2");
    public static final LoggedTunableNumber pivotMMJerkRotPerSec3 = 
        new LoggedTunableNumber("Intake/Pivot/MMJerkRotPerSec3");

    // Slot0 PID
    public static final LoggedTunableNumber pivotkP = 
        new LoggedTunableNumber("Intake/Pivot/kP");
    public static final LoggedTunableNumber pivotkI = 
        new LoggedTunableNumber("Intake/Pivot/kI");
    public static final LoggedTunableNumber pivotkD = 
        new LoggedTunableNumber("Intake/Pivot/kD");
    public static final LoggedTunableNumber pivotkS = 
        new LoggedTunableNumber("Intake/Pivot/kS");
    public static final LoggedTunableNumber pivotkV = 
        new LoggedTunableNumber("Intake/Pivot/kV");
    public static final LoggedTunableNumber pivotkG = 
        new LoggedTunableNumber("Intake/Pivot/kG");

    // Game piece detection
    public static final LoggedTunableNumber gamePieceCurrentThreshold = 
        new LoggedTunableNumber("Intake/GamePieceCurrentThreshold");

    // ===================== Crash / stall detection =====================
    public static final LoggedTunableNumber pivotCrashCurrentAmps = 
        new LoggedTunableNumber("Intake/Pivot/CrashCurrentAmps");
    public static final LoggedTunableNumber pivotCrashMinErrorRot = 
        new LoggedTunableNumber("Intake/Pivot/CrashMinErrorRot");
    public static final LoggedTunableNumber pivotCrashMaxVelRotPerSec = 
        new LoggedTunableNumber("Intake/Pivot/CrashMaxVelRotPerSec");
    public static final LoggedTunableNumber pivotCrashDebounceSecs = 
        new LoggedTunableNumber("Intake/Pivot/CrashDebounceSecs");
    public static final LoggedTunableNumber pivotAllowedErrorRot = 
        new LoggedTunableNumber("Intake/Pivot/AllowedErrorRot");
    public static final LoggedTunableNumber pivotAllowedVelRotPerSec = 
        new LoggedTunableNumber("Intake/Pivot/AllowedVelRotPerSec");
    public static final LoggedTunableNumber pivotCrashIgnoreAfterGoalChangeSecs = 
        new LoggedTunableNumber("Intake/Pivot/CrashIgnoreAfterGoalChangeSecs");

    // Static initializer - sets defaults based on robot type
    static {
        switch (Constants.getRobot()) {
            case SIMBOT -> {
                // Pivot Motion Magic
                pivotSensorToMechanismRatio.initDefault(12);
                pivotStowedPosRot.initDefault(0);
                pivotDeployedPosRot.initDefault(0.215);
                pivotPosToleranceRot.initDefault(0.01);
                pivotVelToleranceRotPerSec.initDefault(0.5);

                // Motion Magic constraints
                pivotMMCruiseVelRotPerSec.initDefault(15);
                pivotMMAccelRotPerSec2.initDefault(30);
                pivotMMJerkRotPerSec3.initDefault(0);

                // PID
                pivotkP.initDefault(100);
                pivotkI.initDefault(0.1);
                pivotkD.initDefault(0.0);
                pivotkS.initDefault(0.0);
                pivotkV.initDefault(0.0);
                pivotkG.initDefault(0.0);

                // Game piece detection
                gamePieceCurrentThreshold.initDefault(25);

                // Crash detection
                pivotCrashCurrentAmps.initDefault(50);
                pivotCrashMinErrorRot.initDefault(0.05);
                pivotCrashMaxVelRotPerSec.initDefault(0.1);
                pivotCrashDebounceSecs.initDefault(0.15);
                pivotAllowedErrorRot.initDefault(0.01);
                pivotAllowedVelRotPerSec.initDefault(0.5);
                pivotCrashIgnoreAfterGoalChangeSecs.initDefault(0.5);
            }
            case COMPBOT -> {
                // Pivot Motion Magic
                pivotSensorToMechanismRatio.initDefault(12);  // TODO: verify gear ratio
                pivotStowedPosRot.initDefault(0);
                pivotDeployedPosRot.initDefault(0);  // TODO: tune on real robot
                pivotPosToleranceRot.initDefault(0.01);
                pivotVelToleranceRotPerSec.initDefault(0.05);

                // Motion Magic constraints
                pivotMMCruiseVelRotPerSec.initDefault(0);  // TODO
                pivotMMAccelRotPerSec2.initDefault(0);  // TODO
                pivotMMJerkRotPerSec3.initDefault(0);  // TODO

                // PID
                pivotkP.initDefault(0);  // TODO
                pivotkI.initDefault(0);
                pivotkD.initDefault(0);  // TODO
                pivotkS.initDefault(0);  // TODO
                pivotkV.initDefault(0);  // TODO
                pivotkG.initDefault(0);  // TODO

                // Game piece detection
                gamePieceCurrentThreshold.initDefault(25);

                // Crash detection
                pivotCrashCurrentAmps.initDefault(50);  // TODO
                pivotCrashMinErrorRot.initDefault(0.05);  // TODO
                pivotCrashMaxVelRotPerSec.initDefault(0.1);  // TODO
                pivotCrashDebounceSecs.initDefault(0.15);  // TODO
                pivotAllowedErrorRot.initDefault(0.01);
                pivotAllowedVelRotPerSec.initDefault(0.5);
                pivotCrashIgnoreAfterGoalChangeSecs.initDefault(0.5);
            }
        }
    }
}