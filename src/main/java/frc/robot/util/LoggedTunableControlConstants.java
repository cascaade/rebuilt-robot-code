package frc.robot.util;

import java.util.HashSet;
import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedTunableControlConstants {
    private static final Set<LoggedTunableControlConstants> INSTANCES = new HashSet<>();

    private final String baseKey;

    private final LoggedNetworkNumber loggedKP;
    private final LoggedNetworkNumber loggedKI;
    private final LoggedNetworkNumber loggedKD;
    private final LoggedNetworkNumber loggedKS;
    private final LoggedNetworkNumber loggedKV;
    private final LoggedNetworkNumber loggedKCos;

    private double lastKP;
    private double lastKI;
    private double lastKD;
    private double lastKS;
    private double lastKV;
    private double lastKCos;

    private UpdateCallback callback;

    public LoggedTunableControlConstants(String baseKey) {
        if (baseKey.endsWith("/")) {
            baseKey = baseKey.substring(0, baseKey.length() - 1);
        }

        this.baseKey = baseKey;

        loggedKP = new LoggedNetworkNumber(baseKey + "/Tuning/kP");
        loggedKI = new LoggedNetworkNumber(baseKey + "/Tuning/kI");
        loggedKD = new LoggedNetworkNumber(baseKey + "/Tuning/kD");
        loggedKS = new LoggedNetworkNumber(baseKey + "/Tuning/kS");
        loggedKV = new LoggedNetworkNumber(baseKey + "/Tuning/kV");
        loggedKCos = new LoggedNetworkNumber(baseKey + "/Tuning/kCos");

        System.out.println("Control Constants created for " + baseKey);
    }

    public LoggedTunableControlConstants setP(double setValue) {
        loggedKP.set(setValue);
        lastKP = setValue;
        return this;
    }

    public LoggedTunableControlConstants setI(double setValue) {
        loggedKI.set(setValue);
        lastKI = setValue;
        return this;
    }

    public LoggedTunableControlConstants setD(double setValue) {
        loggedKD.set(setValue);
        lastKD = setValue;
        return this;
    }

    public LoggedTunableControlConstants setS(double setValue) {
        loggedKS.set(setValue);
        lastKS = setValue;
        return this;
    }

    public LoggedTunableControlConstants setV(double setValue) {
        loggedKV.set(setValue);
        lastKV = setValue;
        return this;
    }

    public LoggedTunableControlConstants setCos(double setValue) {
        loggedKCos.set(setValue);
        lastKCos = setValue;
        return this;
    }

    public double kP() {
        return lastKP;
    }

    public double kI() {
        return lastKI;
    }

    public double kD() {
        return lastKD;
    }

    public double kS() {
        return lastKS;
    }

    public double kV() {
        return lastKV;
    }

    public double kCos() {
        return lastKCos;
    }

    public void setCallback(UpdateCallback callback) {
        if (this.callback != null)
            System.err.println("Callback for " + baseKey + " tuning constants overwritten.");

        this.callback = callback;

        callback.execute();

        INSTANCES.add(this);
    }

    public void periodic() {
        double currentKP = loggedKP.get();
        double currentKI = loggedKI.get();
        double currentKD = loggedKD.get();
        double currentKS = loggedKS.get();
        double currentKV = loggedKV.get();
        double currentKCos = loggedKCos.get();

        if (
            Math.abs(currentKP - lastKP) > 1e-9 ||
            Math.abs(currentKI - lastKI) > 1e-9 ||
            Math.abs(currentKD - lastKD) > 1e-9 ||
            Math.abs(currentKS - lastKS) > 1e-9 ||
            Math.abs(currentKV - lastKV) > 1e-9 ||
            Math.abs(currentKCos - lastKCos) > 1e-9
        ) {
            lastKP = currentKP;
            lastKI = currentKI;
            lastKD = currentKD;
            lastKS = currentKS;
            lastKV = currentKV;
            lastKCos = currentKCos;

            if (callback == null) {
                throw new Error("LoggedTunableControlConstants changed, but no callback was set.");
            } else {
                callback.execute();
                System.out.println("Control Constants for " + baseKey + " have updated.");
            }
        }
    }

    public static void tuningPeriodic() {
        for (LoggedTunableControlConstants constants : INSTANCES) {
            constants.periodic();
        }
    }

    @FunctionalInterface
    public static interface UpdateCallback {
        void execute();
    }
}