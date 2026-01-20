package frc.robot.subsystems.Index;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.utility.LoggedTunableNumber;

public class IndexConstants {
    public static final int kIndexMotorId = 15;
    
    public static final LoggedTunableNumber indexDutyCycle = new LoggedTunableNumber("Index/DutyCycle");      // Normal indexing speed
    public static final LoggedTunableNumber reverseDutyCycle = new LoggedTunableNumber("Index/ReverseDutyCycle");   // Reverse/eject speed
    
    // CANRange detection threshold in meters - tune on real robot
    public static final LoggedTunableNumber gamePieceDetectionThreshold = new LoggedTunableNumber("Index/GamePieceDetectionThreshold");

    static {
        switch(Constants.getRobot()){
            case SIMBOT -> {
                indexDutyCycle.initDefault(0.8);
                reverseDutyCycle.initDefault(-0.5);
                gamePieceDetectionThreshold.initDefault(Units.inchesToMeters(0.5));
            }
            case COMPBOT -> {
                indexDutyCycle.initDefault(0.8);
                reverseDutyCycle.initDefault(-0.5);
                gamePieceDetectionThreshold.initDefault(Units.inchesToMeters(0.5));
            }

        }
    }
}
