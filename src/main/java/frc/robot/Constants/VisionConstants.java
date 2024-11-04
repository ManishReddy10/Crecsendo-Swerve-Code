package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final String kMainCameraName = ("");
    public static final String kBackCameraName = ("");

    public static final Transform3d kRobotToMainCam = new Transform3d(Units.inchesToMeters(0), Units.inchesToMeters(-4),
            Units.inchesToMeters(46), new Rotation3d(0, 0, 0)); // these are just estimated

    public static final Transform3d kRobotToBackCam = new Transform3d(Units.inchesToMeters(-1), Units.inchesToMeters(6.5),
     Units.inchesToMeters(6.75), new Rotation3d(Units.degreesToRadians(180),Units.degreesToRadians(-34),Units.degreesToRadians(180)));



}
