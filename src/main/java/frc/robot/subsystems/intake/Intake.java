package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.IntakeWristPose;

public class Intake extends SubsystemBase {
    private final RollerIO rollerIO;
    private final WristIO wristIO;

    private boolean isDeployedFlag = true;
    private boolean isHomingFlag = false;
    private boolean isRollingFlag = true;
    private boolean rollingDir = false; // False if not moving in reverse

    private double wristSetpointAdjust = 0.0;

    private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    LoggedNetworkNumber speed = new LoggedNetworkNumber("Intake/Roller/Speed", IntakeConstants.intakeMaxSpeed);
    LoggedNetworkNumber position = new LoggedNetworkNumber("Intake/Wrist/Pose", 0);

    public Intake(RollerIO rollerIO, WristIO wristIO) {
        this.rollerIO = rollerIO;
        this.wristIO = wristIO;
    }

    private void setPose(IntakeWristPose pose) {
        wristIO.setSetpoint(pose.getSetpoint() + wristSetpointAdjust);
    }

    public Command incrementWristSetpointAdjust(boolean positive) {
        return runOnce(() -> {
            if (positive) {
                wristSetpointAdjust += 0.2;
            } else {
                wristSetpointAdjust -= 0.2;
            }
        });
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

    public Command runHomingRoutine() {
        return run(() -> {
            isHomingFlag = true;
            wristIO.setOpenLoop(-2.0);
        })
            .until(() -> wristInputs.currentAmps > IntakeConstants.autoHomeCurrentThreshold)
            .andThen(resetPosition())
            .andThen(() -> {
                isDeployedFlag = false;
                isHomingFlag = false;
            });
    }

    public Command toggleRollerFlag() {
        return runOnce(() -> {
            isRollingFlag = !isRollingFlag;
        });
    }

        public Command toggleRollerDirection() {
        return runOnce(() -> {
            rollingDir = !rollingDir;
        });
    }

    @Override
    public void periodic() {
        if (!isHomingFlag) {
            if (isDeployedFlag) {
                setPose(IntakeWristPose.DEPLOYED);
            } else {
                setPose(IntakeWristPose.STOWED);
            }
        }
        if(isRollingFlag) {
            rollerIO.setClosedLoop(rollingDir ? -speed.get() : speed.get());
        } else {
            rollerIO.setOpenLoop(0);
        }

        Logger.recordOutput("Intake/Deployed", isDeployedFlag);
        Logger.recordOutput("Intake/Homing", isHomingFlag);
        Logger.recordOutput("Intake/RollersRunning", isRollingFlag);
        Logger.recordOutput("Intake/RollerDirection", rollingDir);

        rollerIO.periodic();
        wristIO.periodic();

        wristIO.updateInputs(wristInputs);
        rollerIO.updateInputs(rollerInputs);

        Logger.processInputs("Intake/Wrist", wristInputs);
        Logger.processInputs("Intake/Roller", rollerInputs);
    }
}
