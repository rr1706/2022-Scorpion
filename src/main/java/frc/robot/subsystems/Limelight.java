package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Utilities.LinearInterpolationTable;
import java.awt.geom.Point2D;

//Code here is input from 2020 code and modified for photonvision OS on limelight so angles and distances are left in inches
/**
 * The Limelight class uses a static method to call the functions, because there
 * is only 1 limelight PC and it always outputs to the same Network
 * table the Limelight functions should be called without creating an instance
 * of the object, IE Limelight.tx();
 */
public class Limelight {
    // Output Network Table from the limelight with photonvision OS defined so
    // values can be read by the robot code.
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private static Point2D[] points = new Point2D.Double[] {
            // (ty-angle,distance)
            new Point2D.Double(-22.7, 308.0), // 242
            new Point2D.Double(-20.0, 274.0), // 196
            new Point2D.Double(-17.5, 245.0), // 163
            new Point2D.Double(-15.0, 217.0), // 141
            new Point2D.Double(-12.5, 200.0), // 121
            new Point2D.Double(-10.0, 184.0), // 107
            new Point2D.Double(-5.0, 157.0), // 96
            new Point2D.Double(0.0, 136.0), // 85
            new Point2D.Double(5.0, 120.0), // 77
            new Point2D.Double(10.0, 106.0),
            new Point2D.Double(15.0, 94.0),
            new Point2D.Double(18.5, 86.0)
            //
    };
    private static LinearInterpolationTable distTable = new LinearInterpolationTable(points);

    /**
     * Adds the azimuthal angle to the limelight target yaw angle. Then converts to
     * radians before returning value
     *
     * @return the yaw angle in radians.
     */
    public static double tx() {
        return Math.toRadians(VisionConstants.kAzimuthalAngle + table.getEntry("tx").getDouble(0.0));
    }

    /**
     * the limelight target pitch angle
     *
     * @return the pitch angle in radians.
     */
    public static double ty() {
        return table.getEntry("ty").getDouble(0.0);
    }

    /**
     * Singifies if the limelight currently has an accetable target, defaulting to
     * false if no value is provided in the Network Table.
     * This default is important so that if the limelight becomes disconnected or
     * gives bad numbers the code will assume there is not a valid target
     *
     * @return true if an acceptable target is visible.
     */
    public static boolean valid() {
        return table.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * Allows retreival of the target area.
     *
     * @return the area of the target.
     */
    public static double ta() {
        return table.getEntry("ta").getDouble(0.0);
    }

    /**
     * Uses tuned interpolation table to report distance
     *
     * @return the distance to the target in inches
     */
    public static double getDistance() {
        final double tx = tx()*180.0/Math.PI;
        final double tyAdj = (ty() -0.009742*tx*tx)/(0.00036*tx*tx+1.0); //New geometric correction function
        final double distance = distTable.getOutput(tyAdj);
        //SmartDashboard.putNumber("Limelight ty", ty());
        // SmartDashboard.putNumber("LimelightDistance", distance);
        return distance;
    }

    /**
     * Enables the limelight LED array. It is important to only enable when in use
     * to comply with FRC Game Rules.
     */
    public static void enable() {
        table.getEntry("ledMode").setNumber(0);
    }

    /**
     * Disables the limelight LED array. It is important to disable when not in use
     * to comply with FRC Game Rules.
     */
    public static void disable() {
        table.getEntry("ledMode").setNumber(1);
    }
    
}