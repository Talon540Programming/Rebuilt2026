package frc.robot.subsystems.Climberz;

import frc.robot.Constants;
import frc.robot.utility.LoggedTunableNumber;

public class ClimberzConstants {
    public static final int motorId = 20;
    
    // Duty cycle control (backup/manual)
    public static final double climbUpDutyCycle = 0.8;
    public static final double climbDownDutyCycle = 0.4;
    
    // Homing constants
    public static final double homingDutyCycle = 0.2;
    public static final double homingDurationSeconds = 1.75;
    public static final LoggedTunableNumber homingVelThreshold = 
        new LoggedTunableNumber("Climberz/HomingVelThreshold");
    public static final LoggedTunableNumber homingCurrentThreshold = 
        new LoggedTunableNumber("Climberz/HomingCurrentThreshold");
    public static final LoggedTunableNumber homingTimeoutSeconds = 
        new LoggedTunableNumber("Climberz/HomingTimeoutSeconds");

    // Position control constants
    public static final LoggedTunableNumber retractedPositionRot = 
        new LoggedTunableNumber("Climberz/RetractedPositionRot");
    public static final LoggedTunableNumber extendedPositionRot = 
        new LoggedTunableNumber("Climberz/ExtendedPositionRot");
    public static final LoggedTunableNumber positionToleranceRot = 
        new LoggedTunableNumber("Climberz/PositionToleranceRot");

    // Motion Magic constraints
    public static final LoggedTunableNumber mmCruiseVelRotPerSec = 
        new LoggedTunableNumber("Climberz/MMCruiseVelRotPerSec");
    public static final LoggedTunableNumber mmAccelRotPerSec2 = 
        new LoggedTunableNumber("Climberz/MMAccelRotPerSec2");
    public static final LoggedTunableNumber mmJerkRotPerSec3 = 
        new LoggedTunableNumber("Climberz/MMJerkRotPerSec3");

    // PID gains
    public static final LoggedTunableNumber kP = 
        new LoggedTunableNumber("Climberz/kP");
    public static final LoggedTunableNumber kI = 
        new LoggedTunableNumber("Climberz/kI");
    public static final LoggedTunableNumber kD = 
        new LoggedTunableNumber("Climberz/kD");
    public static final LoggedTunableNumber kS = 
        new LoggedTunableNumber("Climberz/kS");
    public static final LoggedTunableNumber kV = 
        new LoggedTunableNumber("Climberz/kV");
    public static final LoggedTunableNumber kG = 
        new LoggedTunableNumber("Climberz/kG");

    // Gear ratio
    public static final double sensorToMechanismRatio = 48.0;

    static {
        switch (Constants.getRobot()) {
            case SIMBOT -> {
                // Homing
                homingVelThreshold.initDefault(0.1);
                homingCurrentThreshold.initDefault(20.0);
                homingTimeoutSeconds.initDefault(2.0);

                // Position setpoints (TODO: tune)
                retractedPositionRot.initDefault(0.0);
                extendedPositionRot.initDefault(10.0);  // TODO: measure actual travel
                positionToleranceRot.initDefault(0.5);

                // Motion Magic
                mmCruiseVelRotPerSec.initDefault(40.0);
                mmAccelRotPerSec2.initDefault(80.0);
                mmJerkRotPerSec3.initDefault(0.0);

                // PID
                kP.initDefault(5.0);
                kI.initDefault(0.0);
                kD.initDefault(0.1);
                kS.initDefault(0.0);
                kV.initDefault(0.12);
                kG.initDefault(0.0);  // TODO: tune for gravity
            }
            case COMPBOT -> {
                // Homing
                homingVelThreshold.initDefault(0.1);  // TODO: tune
                homingCurrentThreshold.initDefault(20.0);  // TODO: tune
                homingTimeoutSeconds.initDefault(2.0);

                // Position setpoints (TODO: tune)
                retractedPositionRot.initDefault(0.0);
                extendedPositionRot.initDefault(10.0);  // TODO: measure actual travel
                positionToleranceRot.initDefault(0.5);

                // Motion Magic
                mmCruiseVelRotPerSec.initDefault(40.0);  // TODO: tune
                mmAccelRotPerSec2.initDefault(80.0);  // TODO: tune
                mmJerkRotPerSec3.initDefault(0.0);

                // PID
                kP.initDefault(0.0);  // TODO: tune
                kI.initDefault(0.0);
                kD.initDefault(0.0);  // TODO: tune
                kS.initDefault(0.0);  // TODO: tune
                kV.initDefault(0.0);  // TODO: tune
                kG.initDefault(0.0);  // TODO: tune for gravity
            }
        }
    }
}