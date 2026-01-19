package frc.robot.subsystems.Intake;

import frc.robot.Constants;
import frc.robot.utility.LoggedTunableNumber;

public class IntakeConstants {
    // Shared constants (same for sim and real)
    public static final int extensionMotorId = 13;
    public static final int rollerMotorId = 14;

    public static final double extensionHomingDutyCycle = -0.15;
    public static final double extensionHomingVelThreshold = 0.05;
    public static final double extensionHomingCurrentThreshold = 5.0;

    public static final double dployDutyCycle = 0.5;
    public static final double rollerIntakeDutyCycle = 0.8;
    public static final double rollerEjectDutyCycle = -0.5;

    // ===================== Pivot Motion Magic =====================
    public static final LoggedTunableNumber extensionSensorToMechanismRatio = 
        new LoggedTunableNumber("Intake/Extension/SensorToMechanismRatio");
    public static final LoggedTunableNumber extensionStowedPosRot = 
        new LoggedTunableNumber("Intake/Extension/StowedPosRot");
    public static final LoggedTunableNumber extensionDeployedPosRot = 
        new LoggedTunableNumber("Intake/Extension/DeployedPosRot");
    public static final LoggedTunableNumber extensionPosToleranceRot = 
        new LoggedTunableNumber("Intake/Extension/PosToleranceRot");
    public static final LoggedTunableNumber extensionVelToleranceRotPerSec = 
        new LoggedTunableNumber("Intake/Extension/VelToleranceRotPerSec");

    // Motion Magic constraints
    public static final LoggedTunableNumber extensionMMCruiseVelRotPerSec = 
        new LoggedTunableNumber("Intake/Extension/MMCruiseVelRotPerSec");
    public static final LoggedTunableNumber extensionMMAccelRotPerSec2 = 
        new LoggedTunableNumber("Intake/Extension/MMAccelRotPerSec2");
    public static final LoggedTunableNumber extensionMMJerkRotPerSec3 = 
        new LoggedTunableNumber("Intake/Extension/MMJerkRotPerSec3");

    // Slot0 PID
    public static final LoggedTunableNumber extensionkP = 
        new LoggedTunableNumber("Intake/Extension/kP");
    public static final LoggedTunableNumber extensionkI = 
        new LoggedTunableNumber("Intake/Extension/kI");
    public static final LoggedTunableNumber extensionkD = 
        new LoggedTunableNumber("Intake/Extension/kD");
    public static final LoggedTunableNumber extensionkS = 
        new LoggedTunableNumber("Intake/Extension/kS");
    public static final LoggedTunableNumber extensionkV = 
        new LoggedTunableNumber("Intake/Extension/kV");
    public static final LoggedTunableNumber extensionkG = 
        new LoggedTunableNumber("Intake/Extension/kG");

    // Game piece detection
    public static final LoggedTunableNumber gamePieceCurrentThreshold = 
        new LoggedTunableNumber("Intake/GamePieceCurrentThreshold");

    // ===================== Crash / stall detection =====================
    public static final LoggedTunableNumber extensionCrashCurrentAmps = 
        new LoggedTunableNumber("Intake/Extension/CrashCurrentAmps");
    public static final LoggedTunableNumber extensionCrashMinErrorRot = 
        new LoggedTunableNumber("Intake/Extension/CrashMinErrorRot");
    public static final LoggedTunableNumber extensionCrashMaxVelRotPerSec = 
        new LoggedTunableNumber("Intake/Extension/CrashMaxVelRotPerSec");
    public static final LoggedTunableNumber extensionCrashDebounceSecs = 
        new LoggedTunableNumber("Intake/Extension/CrashDebounceSecs");
    public static final LoggedTunableNumber extensionAllowedErrorRot = 
        new LoggedTunableNumber("Intake/Extension/AllowedErrorRot");
    public static final LoggedTunableNumber extensionAllowedVelRotPerSec = 
        new LoggedTunableNumber("Intake/Extension/AllowedVelRotPerSec");
    public static final LoggedTunableNumber extensionCrashIgnoreAfterGoalChangeSecs = 
        new LoggedTunableNumber("Intake/Extension/CrashIgnoreAfterGoalChangeSecs");

    // Static initializer - sets defaults based on robot type
    static {
        switch (Constants.getRobot()) {
            case SIMBOT -> {
                // Pivot Motion Magic
                extensionSensorToMechanismRatio.initDefault(12);
                extensionStowedPosRot.initDefault(0);
                extensionDeployedPosRot.initDefault(0.215);
                extensionPosToleranceRot.initDefault(0.01);
                extensionVelToleranceRotPerSec.initDefault(0.5);

                // Motion Magic constraints
                extensionMMCruiseVelRotPerSec.initDefault(15);
                extensionMMAccelRotPerSec2.initDefault(30);
                extensionMMJerkRotPerSec3.initDefault(0);

                // PID
                extensionkP.initDefault(5);
                extensionkI.initDefault(0.1);
                extensionkD.initDefault(0.0);
                extensionkS.initDefault(0.0);
                extensionkV.initDefault(0.0);
                extensionkG.initDefault(0.0);

                // Game piece detection
                gamePieceCurrentThreshold.initDefault(25);

                // Crash detection
                extensionCrashCurrentAmps.initDefault(50);
                extensionCrashMinErrorRot.initDefault(0.05);
                extensionCrashMaxVelRotPerSec.initDefault(0.1);
                extensionCrashDebounceSecs.initDefault(0.15);
                extensionAllowedErrorRot.initDefault(0.01);
                extensionAllowedVelRotPerSec.initDefault(0.5);
                extensionCrashIgnoreAfterGoalChangeSecs.initDefault(0.5);
            }
            case COMPBOT -> {
                // Pivot Motion Magic
                extensionSensorToMechanismRatio.initDefault(12);  // TODO: verify gear ratio
                extensionStowedPosRot.initDefault(0);
                extensionDeployedPosRot.initDefault(0);  // TODO: tune on real robot
                extensionPosToleranceRot.initDefault(0.01);
                extensionVelToleranceRotPerSec.initDefault(0.05);

                // Motion Magic constraints
                extensionMMCruiseVelRotPerSec.initDefault(0);  // TODO
                extensionMMAccelRotPerSec2.initDefault(0);  // TODO
                extensionMMJerkRotPerSec3.initDefault(0);  // TODO

                // PID
                extensionkP.initDefault(0);  // TODO
                extensionkI.initDefault(0);
                extensionkD.initDefault(0);  // TODO
                extensionkS.initDefault(0);  // TODO
                extensionkV.initDefault(0);  // TODO
                extensionkG.initDefault(0);  // TODO

                // Game piece detection
                gamePieceCurrentThreshold.initDefault(25);

                // Crash detection
                extensionCrashCurrentAmps.initDefault(50);  // TODO
                extensionCrashMinErrorRot.initDefault(0.05);  // TODO
                extensionCrashMaxVelRotPerSec.initDefault(0.1);  // TODO
                extensionCrashDebounceSecs.initDefault(0.15);  // TODO
                extensionAllowedErrorRot.initDefault(0.01);
                extensionAllowedVelRotPerSec.initDefault(0.5);
                extensionCrashIgnoreAfterGoalChangeSecs.initDefault(0.5);
            }
        }
    }
}