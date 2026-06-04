package frc.robot.util;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
import lombok.Getter;

import java.util.function.Consumer;

public class TunableControlConstants {
    public enum FeedforwardType {
        NONE,
        SIMPLE,
        ELEVATOR,
        ARM
    }

    private final LoggedTunableNumber kP;
    private final LoggedTunableNumber kI;
    private final LoggedTunableNumber kD;
    private final LoggedTunableNumber kS;
    private final LoggedTunableNumber kV;
    private final LoggedTunableNumber kA;
    private final LoggedTunableNumber kG;

    private final LoggedTunableNumber simKP;
    private final LoggedTunableNumber simKI;
    private final LoggedTunableNumber simKD;
    private final LoggedTunableNumber simKS;
    private final LoggedTunableNumber simKV;
    private final LoggedTunableNumber simKA;
    private final LoggedTunableNumber simKG;

    @Getter
    private FeedforwardType feedforwardType = FeedforwardType.NONE;
    @Getter
    private FeedforwardType simFeedforwardType = FeedforwardType.NONE;
    private final int changeId = hashCode();

    /**
     * @param prefix The logged tunable key prefix (e.g., "Shooter/Flywheel")
     */
    public TunableControlConstants(String prefix) {
        kP = new LoggedTunableNumber(NetworkTablesPathUtil.join(prefix, "kP"));
        kI = new LoggedTunableNumber(NetworkTablesPathUtil.join(prefix, "kI"));
        kD = new LoggedTunableNumber(NetworkTablesPathUtil.join(prefix, "kD"));
        kS = new LoggedTunableNumber(NetworkTablesPathUtil.join(prefix, "kS"));
        kV = new LoggedTunableNumber(NetworkTablesPathUtil.join(prefix, "kV"));
        kA = new LoggedTunableNumber(NetworkTablesPathUtil.join(prefix, "kA"));
        kG = new LoggedTunableNumber(NetworkTablesPathUtil.join(prefix, "kG"));

        simKP = new LoggedTunableNumber(NetworkTablesPathUtil.join(prefix, "Sim", "kP"));
        simKI = new LoggedTunableNumber(NetworkTablesPathUtil.join(prefix, "Sim", "kI"));
        simKD = new LoggedTunableNumber(NetworkTablesPathUtil.join(prefix, "Sim", "kD"));
        simKS = new LoggedTunableNumber(NetworkTablesPathUtil.join(prefix, "Sim", "kS"));
        simKV = new LoggedTunableNumber(NetworkTablesPathUtil.join(prefix, "Sim", "kV"));
        simKA = new LoggedTunableNumber(NetworkTablesPathUtil.join(prefix, "Sim", "kA"));
        simKG = new LoggedTunableNumber(NetworkTablesPathUtil.join(prefix, "Sim", "kG"));
    }

    public TunableControlConstants(String prefix, double p, double i, double d, double ff) {
        this(prefix);
        withP(p).withI(i).withD(d).withSimpleFeedforward(0.0, ff, 0.0);
    }

    public TunableControlConstants withP(double defaultValue) {
        kP.initDefault(defaultValue);
        return this;
    }

    public TunableControlConstants withP(double defaultValue, boolean simOnly) {
        return simOnly ? withSimP(defaultValue) : withP(defaultValue);
    }

    public TunableControlConstants withI(double defaultValue) {
        kI.initDefault(defaultValue);
        return this;
    }

    public TunableControlConstants withI(double defaultValue, boolean simOnly) {
        return simOnly ? withSimI(defaultValue) : withI(defaultValue);
    }

    public TunableControlConstants withD(double defaultValue) {
        kD.initDefault(defaultValue);
        return this;
    }

    public TunableControlConstants withD(double defaultValue, boolean simOnly) {
        return simOnly ? withSimD(defaultValue) : withD(defaultValue);
    }

    public TunableControlConstants withS(double defaultValue) {
        kS.initDefault(defaultValue);
        return this;
    }

    public TunableControlConstants withS(double defaultValue, boolean simOnly) {
        return simOnly ? withSimS(defaultValue) : withS(defaultValue);
    }

    public TunableControlConstants withV(double defaultValue) {
        kV.initDefault(defaultValue);
        return this;
    }

    public TunableControlConstants withV(double defaultValue, boolean simOnly) {
        return simOnly ? withSimV(defaultValue) : withV(defaultValue);
    }

    public TunableControlConstants withA(double defaultValue) {
        kA.initDefault(defaultValue);
        return this;
    }

    public TunableControlConstants withA(double defaultValue, boolean simOnly) {
        return simOnly ? withSimA(defaultValue) : withA(defaultValue);
    }

    public TunableControlConstants withG(double defaultValue) {
        kG.initDefault(defaultValue);
        return this;
    }

    public TunableControlConstants withG(double defaultValue, boolean simOnly) {
        return simOnly ? withSimG(defaultValue) : withG(defaultValue);
    }

    public TunableControlConstants withSimP(double defaultValue) {
        initSimDefault(simKP, defaultValue);
        return this;
    }

    public TunableControlConstants withSimI(double defaultValue) {
        initSimDefault(simKI, defaultValue);
        return this;
    }

    public TunableControlConstants withSimD(double defaultValue) {
        initSimDefault(simKD, defaultValue);
        return this;
    }

    public TunableControlConstants withSimS(double defaultValue) {
        initSimDefault(simKS, defaultValue);
        return this;
    }

    public TunableControlConstants withSimV(double defaultValue) {
        initSimDefault(simKV, defaultValue);
        return this;
    }

    public TunableControlConstants withSimA(double defaultValue) {
        initSimDefault(simKA, defaultValue);
        return this;
    }

    public TunableControlConstants withSimG(double defaultValue) {
        initSimDefault(simKG, defaultValue);
        return this;
    }

    public TunableControlConstants withSimpleFeedforward(double kS, double kV, double kA) {
        feedforwardType = FeedforwardType.SIMPLE;
        return withS(kS).withV(kV).withA(kA);
    }

    public TunableControlConstants withElevatorFeedforward(double kS, double kG, double kV, double kA) {
        feedforwardType = FeedforwardType.ELEVATOR;
        return withS(kS).withG(kG).withV(kV).withA(kA);
    }

    public TunableControlConstants withArmFeedforward(double kS, double kG, double kV, double kA) {
        feedforwardType = FeedforwardType.ARM;
        return withS(kS).withG(kG).withV(kV).withA(kA);
    }

    public TunableControlConstants withSimSimpleFeedforward(double kS, double kV, double kA) {
        simFeedforwardType = FeedforwardType.SIMPLE;
        return withSimS(kS).withSimV(kV).withSimA(kA);
    }

    public TunableControlConstants withSimElevatorFeedforward(double kS, double kG, double kV, double kA) {
        simFeedforwardType = FeedforwardType.ELEVATOR;
        return withSimS(kS).withSimG(kG).withSimV(kV).withSimA(kA);
    }

    public TunableControlConstants withSimArmFeedforward(double kS, double kG, double kV, double kA) {
        simFeedforwardType = FeedforwardType.ARM;
        return withSimS(kS).withSimG(kG).withSimV(kV).withSimA(kA);
    }

    /** Checks if values were updated on the dashboard. */
    public boolean hasChanged() {
        return Constants.tuningMode
            && (kP.hasChanged(changeId)
                | kI.hasChanged(changeId)
                | kD.hasChanged(changeId)
                | kS.hasChanged(changeId)
                | kV.hasChanged(changeId)
                | kA.hasChanged(changeId)
                | kG.hasChanged(changeId));
    }

    /** Checks if simulation-only values were updated on the dashboard. */
    public boolean hasSimChanged() {
        return Constants.currentMode == Constants.RobotMode.SIM
            && Constants.tuningMode
            && (simKP.hasChanged(changeId)
                | simKI.hasChanged(changeId)
                | simKD.hasChanged(changeId)
                | simKS.hasChanged(changeId)
                | simKV.hasChanged(changeId)
                | simKA.hasChanged(changeId)
                | simKG.hasChanged(changeId));
    }

    public boolean haveAnyChanged() {
        return hasChanged() || hasSimChanged();
    }

    public boolean ifSimChanged(Runnable action) {
        if (!hasSimChanged()) {
            return false;
        }

        action.run();
        return true;
    }

    public boolean ifSimChanged(Consumer<TunableControlConstants> action) {
        if (!hasSimChanged()) {
            return false;
        }

        action.accept(this);
        return true;
    }

    public TalonFXConfiguration applyTo(TalonFXConfiguration config) {
        config.Slot0.kP = getP();
        config.Slot0.kI = getI();
        config.Slot0.kD = getD();
        config.Slot0.kS = getS();
        config.Slot0.kV = getV();
        config.Slot0.kA = getA();
        config.Slot0.kG = getG();

        if (feedforwardType == FeedforwardType.ELEVATOR) {
            config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        } else if (feedforwardType == FeedforwardType.ARM) {
            config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        }

        return config;
    }

    public SparkMaxConfig applyTo(SparkMaxConfig config) {
        config.closedLoop.pid(getP(), getI(), getD());
        config.closedLoop.feedForward
            .kS(getS())
            .kV(getV())
            .kA(getA());

        if (feedforwardType == FeedforwardType.ARM) {
            config.closedLoop.feedForward.kCos(getG());
        } else {
            config.closedLoop.feedForward.kG(getG());
        }

        return config;
    }

    public void applyTo(PIDController controller) {
        controller.setPID(getP(), getI(), getD());
    }

    public void applySimTo(PIDController controller) {
        controller.setPID(getSimP(), getSimI(), getSimD());
    }

    public void applySimTo(PIDController controller, SimpleMotorFeedforward ff) {
        controller.setPID(getSimP(), getSimI(), getSimD());
        ff.setKa(getSimA());
        ff.setKs(getSimS());
        ff.setKv(getSimV());
    }

    public boolean applyIfChanged(TalonFXConfiguration config) {
        if (!hasChanged()) {
            return false;
        }

        applyTo(config);
        return true;
    }

    public boolean applyIfChanged(TalonFXConfiguration config, Consumer<TalonFXConfiguration> applyConfig) {
        if (!hasChanged()) {
            return false;
        }

        applyTo(config);
        applyConfig.accept(config);
        return true;
    }

    public boolean applyIfChanged(TalonFXConfiguration config, TalonFX motor) {
        return applyIfChanged(config, c -> motor.getConfigurator().apply(c));
    }

    public boolean applyIfChanged(SparkMaxConfig config) {
        if (!hasChanged()) {
            return false;
        }

        applyTo(config);
        return true;
    }

    public boolean applyIfChanged(SparkMaxConfig config, Consumer<SparkMaxConfig> applyConfig) {
        if (!hasChanged()) {
            return false;
        }

        applyTo(config);
        applyConfig.accept(config);
        return true;
    }

    public boolean applyIfChanged(SparkMaxConfig config, SparkMax motor) {
        return applyIfChanged(config,
            c -> motor.configure(c, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    /*
    override comments for PIDController applyIfChanged:
        "changed" can mean one of three things: any value has changed, only primary values have changed (used for
        physical motors), or the simulation values have changed. PIDController can represent any of these scenarios,
        unlike designated physical PID controller constant classes like SparkMaxConfig or TalonFXConfiguration. to
        account for this, I have created three overrides, one for each of the meanings of "changed".
        the wording of the last one (applySimIfChanged) is different from the second simply because I realized it was
        setting the primary values when the sim was changed, so to clarify i changed it. feel free to change it later
     */

    /**
     * Apply constants to a PID controller if any primary value or any simulation value has changed
     * Applies the constants that correspond to the current robotMode
     * @param config the controller to apply constants to
     * @return if any value has changed and the constants have been applied
     */
    public boolean applyIfChanged(PIDController config) {
        if (!haveAnyChanged()) {
            return false;
        }

        if (Constants.currentMode == RobotMode.SIM)
            applySimTo(config);
        else
            applyTo(config);
        return true;
    }

    /**
     * Apply constants to a PID controller if any primary value has changed
     * @param config the controller to apply constants to
     * @return if any value has changed and the constants have been applied
     */
    public boolean applyIfPrimaryChanged(PIDController config) {
        if (!haveAnyChanged()) {
            return false;
        }

        applyTo(config);
        return true;
    }

    /**
     * Apply constants to a PID controller if any simulation value has changed
     * @param config the controller to apply constants to
     * @return if any value has changed and the constants have been applied
     */
    public boolean applySimIfChanged(PIDController config) {
        if (!hasSimChanged()) {
            return false;
        }

        applySimTo(config);
        return true;
    }

    public SimpleMotorFeedforward getSimpleFeedforward() {
        return new SimpleMotorFeedforward(getS(), getV(), getA());
    }

    public ElevatorFeedforward getElevatorFeedforward() {
        return new ElevatorFeedforward(getS(), getG(), getV(), getA());
    }

    public ArmFeedforward getArmFeedforward() {
        return new ArmFeedforward(getS(), getG(), getV(), getA());
    }

    public SimpleMotorFeedforward getSimSimpleFeedforward() {
        return new SimpleMotorFeedforward(getSimS(), getSimV(), getSimA());
    }

    public ElevatorFeedforward getSimElevatorFeedforward() {
        return new ElevatorFeedforward(getSimS(), getSimG(), getSimV(), getSimA());
    }

    public ArmFeedforward getSimArmFeedforward() {
        return new ArmFeedforward(getSimS(), getSimG(), getSimV(), getSimA());
    }

    public double getP() {
        return kP.get();
    }

    public double getI() {
        return kI.get();
    }

    public double getD() {
        return kD.get();
    }

    public double getS() {
        return kS.get();
    }

    public double getV() {
        return kV.get();
    }

    public double getA() {
        return kA.get();
    }

    public double getG() {
        return kG.get();
    }

    public double getSimP() {
        return simKP.get();
    }

    public double getSimI() {
        return simKI.get();
    }

    public double getSimD() {
        return simKD.get();
    }

    public double getSimS() {
        return simKS.get();
    }

    public double getSimV() {
        return simKV.get();
    }

    public double getSimA() {
        return simKA.get();
    }

    public double getSimG() {
        return simKG.get();
    }

    private void initSimDefault(LoggedTunableNumber number, double defaultValue) {
        if (Constants.currentMode == Constants.RobotMode.SIM) {
            number.initDefault(defaultValue);
        }
    }
}
