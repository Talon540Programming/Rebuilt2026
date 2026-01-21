package frc.robot.subsystems.Shooter;

import frc.robot.Constants;
import frc.robot.utility.LoggedTunableNumber;

public class ShooterConstants {
    // ===================== CAN IDs =====================
    public static final int flywheelMotor1Id = 17;
    public static final int flywheelMotor2Id = 18;
    public static final int hoodMotorId = 19;

    // ===================== Flywheel Constants =====================
    public static final LoggedTunableNumber flywheelkP = 
        new LoggedTunableNumber("Shooter/Flywheel/kP");
    public static final LoggedTunableNumber flywheelkI = 
        new LoggedTunableNumber("Shooter/Flywheel/kI");
    public static final LoggedTunableNumber flywheelkD = 
        new LoggedTunableNumber("Shooter/Flywheel/kD");
    public static final LoggedTunableNumber flywheelkS = 
        new LoggedTunableNumber("Shooter/Flywheel/kS");
    public static final LoggedTunableNumber flywheelkV = 
        new LoggedTunableNumber("Shooter/Flywheel/kV");
    public static final LoggedTunableNumber flywheelkA = 
        new LoggedTunableNumber("Shooter/Flywheel/kA");

    public static final LoggedTunableNumber flywheelRPMThreshould = 
        new LoggedTunableNumber("Shooter/FlywheelRPMThreshould");
    
    // Motion Magic Velocity constraints (Acceleration and Jerk only - no CruiseVelocity)
    public static final LoggedTunableNumber flywheelMMAccelRotPerSec2 = 
        new LoggedTunableNumber("Shooter/Flywheel/MMAccelRotPerSec2");
    public static final LoggedTunableNumber flywheelMMJerkRotPerSec3 = 
        new LoggedTunableNumber("Shooter/Flywheel/MMJerkRotPerSec3");
    
    // Flywheel tolerances
    public static final LoggedTunableNumber flywheelVelToleranceRPM = 
        new LoggedTunableNumber("Shooter/Flywheel/VelToleranceRPM");
    
    // Current limits
    public static final LoggedTunableNumber flywheelStatorCurrentLimit = 
        new LoggedTunableNumber("Shooter/Flywheel/StatorCurrentLimit");
    public static final LoggedTunableNumber flywheelSupplyCurrentLimit = 
        new LoggedTunableNumber("Shooter/Flywheel/SupplyCurrentLimit");

        // ===================== Hood Homing Constants =====================
    public static final LoggedTunableNumber hoodHomingDutyCycle = 
        new LoggedTunableNumber("Shooter/Hood/HomingDutyCycle");
    public static final LoggedTunableNumber hoodHomingVelThreshold = 
        new LoggedTunableNumber("Shooter/Hood/HomingVelThreshold");
    public static final LoggedTunableNumber hoodHomingCurrentThreshold = 
        new LoggedTunableNumber("Shooter/Hood/HomingCurrentThreshold");

    // ===================== Hood Constants =====================
    public static final LoggedTunableNumber hoodkP = 
        new LoggedTunableNumber("Shooter/Hood/kP");
    public static final LoggedTunableNumber hoodkI = 
        new LoggedTunableNumber("Shooter/Hood/kI");
    public static final LoggedTunableNumber hoodkD = 
        new LoggedTunableNumber("Shooter/Hood/kD");
    public static final LoggedTunableNumber hoodkS = 
        new LoggedTunableNumber("Shooter/Hood/kS");
    public static final LoggedTunableNumber hoodkV = 
        new LoggedTunableNumber("Shooter/Hood/kV");
    public static final LoggedTunableNumber hoodkA = 
        new LoggedTunableNumber("Shooter/Hood/kA");
    
    // Motion Magic Position constraints
    public static final LoggedTunableNumber hoodMMCruiseVelRotPerSec = 
        new LoggedTunableNumber("Shooter/Hood/MMCruiseVelRotPerSec");
    public static final LoggedTunableNumber hoodMMAccelRotPerSec2 = 
        new LoggedTunableNumber("Shooter/Hood/MMAccelRotPerSec2");
    public static final LoggedTunableNumber hoodMMJerkRotPerSec3 = 
        new LoggedTunableNumber("Shooter/Hood/MMJerkRotPerSec3");
    
    public static final LoggedTunableNumber hoodSensorToMechanismRatio = 
        new LoggedTunableNumber("Shooter/Hood/SensorToMechanismRatio");
    
    // Hood tolerances
    public static final LoggedTunableNumber hoodPosToleranceRot = 
        new LoggedTunableNumber("Shooter/Hood/PosToleranceRot");
    public static final LoggedTunableNumber hoodVelToleranceRotPerSec = 
        new LoggedTunableNumber("Shooter/Hood/VelToleranceRotPerSec");
    
    // Current limits
    public static final LoggedTunableNumber hoodStatorCurrentLimit = 
        new LoggedTunableNumber("Shooter/Hood/StatorCurrentLimit");
    public static final LoggedTunableNumber hoodSupplyCurrentLimit = 
        new LoggedTunableNumber("Shooter/Hood/SupplyCurrentLimit");

    // ===================== Kickup Constants =====================
    public static final class KickupConstants {
        public static final int kKickupMotorId = 16;
        
        public static final LoggedTunableNumber feedDutyCycle = 
            new LoggedTunableNumber("Shooter/Kickup/FeedDutyCycle");
        public static final LoggedTunableNumber reverseDutyCycle = 
            new LoggedTunableNumber("Shooter/Kickup/ReverseDutyCycle");
        public static final LoggedTunableNumber statorCurrentLimit = 
            new LoggedTunableNumber("Shooter/Kickup/StatorCurrentLimit");
        public static final LoggedTunableNumber supplyCurrentLimit = 
            new LoggedTunableNumber("Shooter/Kickup/SupplyCurrentLimit");
        
        static {
            switch (Constants.getRobot()) {
                case SIMBOT -> {
                    feedDutyCycle.initDefault(0.8);
                    reverseDutyCycle.initDefault(-0.3);
                    statorCurrentLimit.initDefault(60);
                    supplyCurrentLimit.initDefault(40);
                    flywheelRPMThreshould.initDefault(250);
                    // Hood homing
                    hoodHomingDutyCycle.initDefault(-0.1);  // Small negative voltage toward min position
                    hoodHomingVelThreshold.initDefault(0.05);  // Stall detection velocity threshold
                    hoodHomingCurrentThreshold.initDefault(10.0);  // Stall detection current threshold
                    
                }
                case COMPBOT -> {
                    feedDutyCycle.initDefault(0.8);  // TODO: tune
                    reverseDutyCycle.initDefault(-0.3);  // TODO: tune
                    statorCurrentLimit.initDefault(60);
                    supplyCurrentLimit.initDefault(40);
                    flywheelRPMThreshould.initDefault(250);
                    // Hood homing
                    hoodHomingDutyCycle.initDefault(-0.1);  // TODO: tune
                    hoodHomingVelThreshold.initDefault(0.05);  // TODO: tune
                    hoodHomingCurrentThreshold.initDefault(10.0);  // TODO: tune
                }
            }
        }
    }

    // Static initializer - sets defaults based on robot type
    static {
        switch (Constants.getRobot()) {
            case SIMBOT -> {
                // ===== Flywheel Sim Defaults =====
                flywheelkP.initDefault(0.04);
                flywheelkI.initDefault(0.0);
                flywheelkD.initDefault(0.0);
                flywheelkS.initDefault(0.1);
                flywheelkV.initDefault(0.12);
                flywheelkA.initDefault(0.01);
                
                flywheelMMAccelRotPerSec2.initDefault(400);  // 400 rps/s
                flywheelMMJerkRotPerSec3.initDefault(4000);  // 4000 rps/s/s
                
                flywheelVelToleranceRPM.initDefault(50);
                flywheelStatorCurrentLimit.initDefault(80);
                flywheelSupplyCurrentLimit.initDefault(60);
                
                // ===== Hood Sim Defaults =====
                hoodkP.initDefault(4.8);
                hoodkI.initDefault(0.0);
                hoodkD.initDefault(0.1);
                hoodkS.initDefault(0.25);
                hoodkV.initDefault(0.12);
                hoodkA.initDefault(0.01);
                
                hoodMMCruiseVelRotPerSec.initDefault(80);  // 80 rps cruise
                hoodMMAccelRotPerSec2.initDefault(160);    // 160 rps/s
                hoodMMJerkRotPerSec3.initDefault(1600);    // 1600 rps/s/s
            
                hoodSensorToMechanismRatio.initDefault(50);  // 50:1 gear ratio
                
                hoodPosToleranceRot.initDefault(0.005);  // ~2 degrees
                hoodVelToleranceRotPerSec.initDefault(0.1);
                hoodStatorCurrentLimit.initDefault(40);
                hoodSupplyCurrentLimit.initDefault(30);
            }
            case COMPBOT -> {
                // ===== Flywheel Comp Defaults =====
                flywheelkP.initDefault(0.0);  // TODO: tune with SysId
                flywheelkI.initDefault(0.0);
                flywheelkD.initDefault(0.0);
                flywheelkS.initDefault(0.0);  // TODO: tune with SysId
                flywheelkV.initDefault(0.0);  // TODO: tune with SysId
                flywheelkA.initDefault(0.0);  // TODO: tune with SysId
                
                flywheelMMAccelRotPerSec2.initDefault(400);  // TODO: tune
                flywheelMMJerkRotPerSec3.initDefault(4000);  // TODO: tune
                
                flywheelVelToleranceRPM.initDefault(50);
                flywheelStatorCurrentLimit.initDefault(80);
                flywheelSupplyCurrentLimit.initDefault(60);
                
                // ===== Hood Comp Defaults =====
                hoodkP.initDefault(0.0);  // TODO: tune
                hoodkI.initDefault(0.0);
                hoodkD.initDefault(0.0);  // TODO: tune
                hoodkS.initDefault(0.0);  // TODO: tune
                hoodkV.initDefault(0.0);  // TODO: tune
                hoodkA.initDefault(0.0);  // TODO: tune
                
                hoodMMCruiseVelRotPerSec.initDefault(0);  // TODO: tune
                hoodMMAccelRotPerSec2.initDefault(0);     // TODO: tune
                hoodMMJerkRotPerSec3.initDefault(0);      // TODO: tune
                
                hoodSensorToMechanismRatio.initDefault(50);  // TODO: verify
                
                hoodPosToleranceRot.initDefault(0.005);
                hoodVelToleranceRotPerSec.initDefault(0.1);
                hoodStatorCurrentLimit.initDefault(40);
                hoodSupplyCurrentLimit.initDefault(30);
            }
        }
    }
}