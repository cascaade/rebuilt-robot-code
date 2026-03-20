package frc.robot.util;

import edu.wpi.first.math.util.Units;

import java.util.Map;
import java.util.TreeMap;

public class HubShootLUT {
    private static final TreeMap<Double, Double> flywheelSpeedTable = new TreeMap<>();

    private final static double robotOffsetDistance = Units.inchesToMeters(25.0) / 2;
    private final static double hubOffsetDistance = Units.inchesToMeters(23.0);

    static {    
        flywheelSpeedTable.put(Units.inchesToMeters(90) + robotOffsetDistance + hubOffsetDistance, 395.0 - 10.0);
        flywheelSpeedTable.put(Units.inchesToMeters(80) + robotOffsetDistance + hubOffsetDistance, 380.0 - 10.0);
        flywheelSpeedTable.put(Units.inchesToMeters(70) + robotOffsetDistance + hubOffsetDistance, 360.0 - 10.0);
        flywheelSpeedTable.put(Units.inchesToMeters(60) + robotOffsetDistance + hubOffsetDistance, 345.0 - 10.0);
        flywheelSpeedTable.put(Units.inchesToMeters(50) + robotOffsetDistance + hubOffsetDistance, 325.0 - 10.0);
        flywheelSpeedTable.put(Units.inchesToMeters(40) + robotOffsetDistance + hubOffsetDistance, 310.0 - 10.0);
        flywheelSpeedTable.put(Units.inchesToMeters(30) + robotOffsetDistance + hubOffsetDistance, 290.0 - 10.0);
        flywheelSpeedTable.put(Units.inchesToMeters(20) + robotOffsetDistance + hubOffsetDistance, 275.0 - 10.0);
    }

    public static double getFlywheelSpeedAtDistance(double targetDistance) {
        if (flywheelSpeedTable.isEmpty()) {
            return 0.0;
        }

        Map.Entry<Double, Double> lowerPoint = flywheelSpeedTable.floorEntry(targetDistance);
        Map.Entry<Double, Double> upperPoint = flywheelSpeedTable.ceilingEntry(targetDistance);

        // if the point is out of bounds, return nearest point
        if (lowerPoint == null) return upperPoint.getValue();
        if (upperPoint == null) return lowerPoint.getValue();

        // if the target distance was directly in the data, and hence the two nearest points are the same, return one of the two
        if (lowerPoint.getKey().equals(upperPoint.getKey())) return lowerPoint.getValue();

        // standard interpolation
        double x1 = lowerPoint.getKey();
        double y1 = lowerPoint.getValue();
        double x2 = upperPoint.getKey();
        double y2 = upperPoint.getValue();

        return y1 + (targetDistance - x1) * ((y2 - y1) / (x2 - x1));
    }
}
