package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightInterface {
 
    public static double getTX() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    public static double getTY() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    }
}
