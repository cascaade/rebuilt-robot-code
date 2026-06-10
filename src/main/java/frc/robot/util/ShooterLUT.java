package frc.robot.util;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import java.util.Map;
import java.util.TreeMap;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 * A lookup-table that linearly links shooter distance and required speed
 */
public class ShooterLUT {
    private static final TreeMap<Double, Double> flywheelSpeedTable = new TreeMap<>();

    // offsets to make it as easy as plugging in the number from the tape measure into the .put spam below
    private final static double robotOffsetDistance = Units.inchesToMeters(25.0) / 2;
    private final static double hubOffsetDistance = Units.inchesToMeters(23.0);

    static {
        // had to take off a bit of speed because it was overshooting, varying at different distances
        flywheelSpeedTable.put(Units.inchesToMeters(90) + robotOffsetDistance + hubOffsetDistance, 395.0 - 24.0);
        flywheelSpeedTable.put(Units.inchesToMeters(80) + robotOffsetDistance + hubOffsetDistance, 380.0 - 24.0);
        flywheelSpeedTable.put(Units.inchesToMeters(70) + robotOffsetDistance + hubOffsetDistance, 360.0 - 24.0);
        flywheelSpeedTable.put(Units.inchesToMeters(60) + robotOffsetDistance + hubOffsetDistance, 345.0 - 14.0);
        flywheelSpeedTable.put(Units.inchesToMeters(50) + robotOffsetDistance + hubOffsetDistance, 325.0 - 14.0);
        flywheelSpeedTable.put(Units.inchesToMeters(40) + robotOffsetDistance + hubOffsetDistance, 310.0 - 14.0);
        flywheelSpeedTable.put(Units.inchesToMeters(30) + robotOffsetDistance + hubOffsetDistance, 290.0 - 14.0);
        flywheelSpeedTable.put(Units.inchesToMeters(20) + robotOffsetDistance + hubOffsetDistance, 275.0 - 14.0);
    }

    /**
     *
     * @param targetDistance the distance the robot currently is from the hub (center-center)
     * @return the speed the shooter must run at to make a shot into the hub
     */
    public static AngularVelocity getFlywheelSpeedAtDistance(Distance targetDistance) {
        if (flywheelSpeedTable.isEmpty()) {
            return RadiansPerSecond.of(0.0);
        }

        Map.Entry<Double, Double> lowerPoint = flywheelSpeedTable.floorEntry(targetDistance.in(Meters));
        Map.Entry<Double, Double> upperPoint = flywheelSpeedTable.ceilingEntry(targetDistance.in(Meters));

        // if the point is out of bounds, return nearest point
        if (lowerPoint == null) return RadiansPerSecond.of(upperPoint.getValue());
        if (upperPoint == null) return RadiansPerSecond.of(lowerPoint.getValue());

        // if the target distance was directly in the data, and hence the two nearest points are the same, return one of the two
        if (lowerPoint.getKey().equals(upperPoint.getKey())) return RadiansPerSecond.of(lowerPoint.getValue());

        // standard interpolation
        double x1 = lowerPoint.getKey();
        double y1 = lowerPoint.getValue();
        double x2 = upperPoint.getKey();
        double y2 = upperPoint.getValue();

        return RadiansPerSecond.of(y1 + (targetDistance.in(Meters) - x1) * ((y2 - y1) / (x2 - x1)));
    }
}
