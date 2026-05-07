package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXCompanion {
    private TalonFX motor;
    private final TalonFXConfiguration defaultConfig;
    private final TunableNumber kP, kI, kD, kS, kV, kG, kA;

    public TalonFXCompanion(TalonFXConfiguration defaultConfig, String key) {
        this.defaultConfig = defaultConfig;

        this.kP = new TunableNumber(NetworkTablesPathUtil.join(key, "kP"));
        this.kI = new TunableNumber(NetworkTablesPathUtil.join(key, "kI"));
        this.kD = new TunableNumber(NetworkTablesPathUtil.join(key, "kD"));
        this.kS = new TunableNumber(NetworkTablesPathUtil.join(key, "kS"));
        this.kV = new TunableNumber(NetworkTablesPathUtil.join(key, "kV"));
        this.kG = new TunableNumber(NetworkTablesPathUtil.join(key, "kG"));
        this.kA = new TunableNumber(NetworkTablesPathUtil.join(key, "kA"));
    }

    public void setMotor(TalonFX motor) {
        if (this.motor == null) {
            this.motor = motor;
        } else {
            throw new RuntimeException("Cannot set final field \"motor\"");
        }
    }

    public void update() {
        if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged() || kS.hasChanged() || kS.hasChanged() || kV.hasChanged() || kG.hasChanged() || kA.hasChanged()) {
            defaultConfig.Slot0.kP = kP.get();
            defaultConfig.Slot0.kI = kI.get();
            defaultConfig.Slot0.kD = kD.get();
            defaultConfig.Slot0.kS = kS.get();
            defaultConfig.Slot0.kV = kV.get();
            defaultConfig.Slot0.kG = kG.get();
            defaultConfig.Slot0.kA = kA.get();

            motor.getConfigurator().apply(defaultConfig);
        }
    }
}
