package frc.robot.subsystems.LED;

import com.ctre.phoenix6.signals.RGBWColor;

public class LEDConstants {
    // CAN ID
    public static final int kCANdleId = 21;
    
    // LED Configuration
    public static final int kLEDCount = 64 + 7; // TODO: Update with actual LED count 7 included LEDs accout for this 
    public static final int kCANdleBuiltInLEDs = 8; // CANdle has 8 built-in LEDs
    public static final int kTotalLEDs = kLEDCount + kCANdleBuiltInLEDs;
    
    // Animation Speeds (0.0 to 1.0)
    public static final double kStrobeSpeed = 0.95; // 10 flashes per second (fast as possible)
    public static final double kFlowSpeed = 0.7;    // Flowing animation speed
    public static final double kFireSpeed = 0.5;    // Fire animation speed
    public static final double flickerSpeed = 0.1;
    
    // Brightness (0.0 to 1.0)
    public static final double kBrightness = 1.0;
    
    // Colors (R, G, B)
    public static final RGBWColor RED = new RGBWColor(255,0,0,0); 
    public static final RGBWColor BLUE = new RGBWColor(0,0,255,0); 
    public static final RGBWColor WHITE = new RGBWColor(255,255,255,0);
    public static final RGBWColor YELLOW = new RGBWColor(255,255,0,0);
    public static final RGBWColor ORANGE = new RGBWColor(255,165,0,0);
    public static final RGBWColor CYAN = new RGBWColor(0,255,255,0);
    public static final RGBWColor GREEN = new RGBWColor(0,255,0,0);
}