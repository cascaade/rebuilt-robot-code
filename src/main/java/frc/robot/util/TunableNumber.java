package frc.robot.util;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunableNumber {
    private static final String tableKey = "/Tuning";

    private final String key;
    private double lastHasChangedValue;

    private final LoggedNetworkNumber dashboardNumber;

    public TunableNumber(String key, double defaultValue) {
        this.key = NetworkTablesPathUtil.join(tableKey, key);

        dashboardNumber = new LoggedNetworkNumber(this.key, defaultValue);
    }

    public TunableNumber(String key) {
        this(key, 0.0);
    }

    public boolean hasChanged() {
        double currentValue = get();
        if (currentValue == lastHasChangedValue) {
            lastHasChangedValue = currentValue;
            return true;
        }
        return false;
    }

    public double get() {
        return dashboardNumber.get();
    }
}
