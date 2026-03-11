package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Constants;
import frc.robot.subsystems.Constants.IntakeConstants;
import frc.robot.subsystems.Constants.IntakeConstants.IntakeWristPose;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Intake extends SubsystemBase {
    private final RollerIO rollerIO;
    private final WristIO wristIO;

    private boolean isDeployedFlag = true;

    private RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
    private WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    LoggedNetworkNumber speed = new LoggedNetworkNumber("Intake/Roller/Speed", IntakeConstants.intakeMaxSpeed);
    LoggedNetworkNumber position = new LoggedNetworkNumber("Intake/Wrist/Pose", 0);

    public Intake(RollerIO rollerIO, WristIO wristIO) {
        this.rollerIO = rollerIO;
        this.wristIO = wristIO;
    }

    private void setPose(IntakeWristPose pose) {
        wristIO.setSetpoint(pose.getSetpoint());
    }

    public Command runRollers() {
        return run(() -> {
            rollerIO.setClosedLoop(speed.get());
        });
    }

    public Command toggleWristPose() {
        return runOnce(() -> {
            isDeployedFlag = !isDeployedFlag;
        });
    }

    public Command runJustWrist() {
        return run(() -> {
//             setPose(poseChooser.get());
//            wristIO.setOpenLoop(-1.0);
            wristIO.setSetpoint(position.get());
        });
    }

    public Command resetPosition() {
        return runOnce(() -> {
            wristIO.resetPosition();
        });
    }

    public Command runStopWrist() {
        return run(() -> {
            wristIO.setOpenLoop(0);
        });
    }

    public Command runStopRollers() {
        return run(() -> {
            rollerIO.setOpenLoop(0);
        });
    }

    @Override
    public void periodic() {
        if (isDeployedFlag) {
            setPose(IntakeWristPose.DEPLOYED);
        } else {
            setPose(IntakeWristPose.STOWED);
        }

        rollerIO.periodic();
        wristIO.periodic();

        wristIO.updateInputs(wristInputs);
        rollerIO.updateInputs(rollerInputs);

        Logger.processInputs("Intake/Wrist", wristInputs);
        Logger.processInputs("Intake/Roller", rollerInputs);
    }
}
