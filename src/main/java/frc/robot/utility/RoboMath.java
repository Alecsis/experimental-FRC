package frc.robot.utility;

import java.util.OptionalDouble;
import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Vision;

public class RoboMath {

    private static final TreeMap<Double, Double> RPMLUT = new TreeMap<>();
    static {
        /* 
        RPMLUT.put(6.5, 2900.0);

        RPMLUT.put(8.4, 3100.0);
        RPMLUT.put(8.9, 2900.0);
        RPMLUT.put(9.2, 3100.0);
        RPMLUT.put(9.3, 3100.0);
        RPMLUT.put(9.5, 3400.0);
        RPMLUT.put(10.6, 3300.0);
        RPMLUT.put(10.7, 3360.0);
        RPMLUT.put(10.7, 3360.0);
        */
        RPMLUT.put(12.4, 1625.0);
        RPMLUT.put(8.8, 1475.0);

        RPMLUT.put(14.5, 2150.0);
        RPMLUT.put(10.6, 1550.0);
        RPMLUT.put(7.5, 1500.0);

        RPMLUT.put(11.3, 1650.0);

        // 12
    }

    public static OptionalDouble calculateRPMFromVision(
            Vision vision,
            Pose2d currentRobotPose) {
        OptionalDouble distanceMetersOpt = vision.hubDistanceMeters(currentRobotPose);

        if (distanceMetersOpt.isEmpty()) {
            return OptionalDouble.empty();
        }

        double distanceFeet = Units.metersToFeet(distanceMetersOpt.getAsDouble());
        double rpm = interpolate(distanceFeet);
        rpm = MathUtil.clamp(rpm, 1200, 6000);
        return OptionalDouble.of(rpm);
    }

    public static double interpolate(double distanceFeet) {
        if (distanceFeet <= RPMLUT.firstKey())
            return RPMLUT.firstEntry().getValue();
        if (distanceFeet >= RPMLUT.lastKey())
            return RPMLUT.lastEntry().getValue();

        var lo = RPMLUT.floorEntry(distanceFeet);
        var hi = RPMLUT.ceilingEntry(distanceFeet);

        double t = (distanceFeet - lo.getKey()) / (hi.getKey() - lo.getKey());
        return lo.getValue() + t * (hi.getValue() - lo.getValue());
    }

}