package frc.robot.util;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMaxConfigurator {
    private final SparkMax motor;
    private final SparkMaxConfig defaultConfig;
    private final TunableNumber kP, kI, kD, kS, kV, kG, kA;

    public SparkMaxConfigurator(SparkMax motor, SparkMaxConfig defaultConfig, String key) {
        this.motor = motor;
        this.defaultConfig = defaultConfig;

        this.kP = new TunableNumber(NetworkTablesPathUtil.join(key, "kP"));
        this.kI = new TunableNumber(NetworkTablesPathUtil.join(key, "kI"));
        this.kD = new TunableNumber(NetworkTablesPathUtil.join(key, "kD"));
        this.kS = new TunableNumber(NetworkTablesPathUtil.join(key, "kS"));
        this.kV = new TunableNumber(NetworkTablesPathUtil.join(key, "kV"));
        this.kG = new TunableNumber(NetworkTablesPathUtil.join(key, "kG"));
        this.kA = new TunableNumber(NetworkTablesPathUtil.join(key, "kA"));
    }

    public void update() {
        if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged() || kS.hasChanged() || kS.hasChanged() || kV.hasChanged() || kG.hasChanged() || kA.hasChanged()) {
            defaultConfig.closedLoop.p(kP.get());
            defaultConfig.closedLoop.i(kI.get());
            defaultConfig.closedLoop.d(kD.get());
            defaultConfig.closedLoop.feedForward.kS(kS.get());
            defaultConfig.closedLoop.feedForward.kV(kV.get());
            defaultConfig.closedLoop.feedForward.kG(kG.get());
            defaultConfig.closedLoop.feedForward.kA(kA.get());
            motor.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }
}
