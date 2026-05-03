package frc.robot.util;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class SparkMaxCompanion {
    private SparkMax motor;
    private final SparkMaxConfig defaultConfig;
    private final TunableNumber kP, kI, kD, kS, kV, kG, kA;

    public SparkMaxCompanion(SparkMaxConfig defaultConfig, String key) {
        this.defaultConfig = defaultConfig;

        this.kP = new TunableNumber(NetworkTablesPathUtil.join(key, "kP"));
        this.kI = new TunableNumber(NetworkTablesPathUtil.join(key, "kI"));
        this.kD = new TunableNumber(NetworkTablesPathUtil.join(key, "kD"));
        this.kS = new TunableNumber(NetworkTablesPathUtil.join(key, "kS"));
        this.kV = new TunableNumber(NetworkTablesPathUtil.join(key, "kV"));
        this.kG = new TunableNumber(NetworkTablesPathUtil.join(key, "kG"));
        this.kA = new TunableNumber(NetworkTablesPathUtil.join(key, "kA"));
    }

    public void setMotor(SparkMax motor) {
        if (this.motor == null) {
            this.motor = motor;
        } else {
            throw new RuntimeException("Cannot set final field \"motor\"");
        }
    }

    public void update() {
        if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged() || kS.hasChanged() || kS.hasChanged() || kV.hasChanged() || kG.hasChanged() || kA.hasChanged()) {
            defaultConfig.closedLoop.pid(kP.get(), kI.get(), kD.get());
            defaultConfig.closedLoop.feedForward.svag(kS.get(), kV.get(), kA.get(), kG.get());

            motor.configure(defaultConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        }
    }
}
