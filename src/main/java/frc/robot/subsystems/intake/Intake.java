package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final RollerIO rollerIO;
    private final WristIO wristIO;

    private boolean isDeployedFlag = true;
    private boolean isHomingFlag = false;
    public boolean isRollingFlag = false;
    private boolean isDirectionReversed = false;

    private boolean isWristMovingUp = false;
    private boolean isWristMovingDown = false;
    private boolean isAutoPulsing = false;

    private Angle pivotSetpointAdjust = Radians.mutable(0.0);

    private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    LoggedNetworkNumber speed = new LoggedNetworkNumber("Intake/Roller/Speed", IntakeConstants.intakeMaxSpeed);
    LoggedNetworkNumber voltage = new LoggedNetworkNumber("Intake/Roller/Voltage", 10);
    LoggedNetworkNumber position = new LoggedNetworkNumber("Intake/Wrist/Pose", 0);

    public Intake(RollerIO rollerIO, WristIO wristIO) {
        this.rollerIO = rollerIO;
        this.wristIO = wristIO;
    }

//    private void setPose(IntakeWristPose pose) {
////        wristIO.setSetpoint(pose.getSetpoint() + wristSetpointAdjust);
//    }
//
//    public Command incrementWristSetpointAdjust(boolean positive) {
//        return runOnce(() -> {
//            if (positive) {
//                wristSetpointAdjust += 0.2;
//            } else {
//                wristSetpointAdjust -= 0.2;
//            }
//        });
//    }

//    public Command runRollers() {
//        return run(() -> {
//            rollerIO.setClosedLoop(speed.get());
//        });
//    }
//
//    public Command toggleWristPose() {
//        return runOnce(() -> {
//            isDeployedFlag = !isDeployedFlag;
//        });
//    }
//
//    public Command runJustWrist() {
//        return run(() -> {
////             setPose(poseChooser.get());
////            wristIO.setOpenLoop(-1.0);
//            wristIO.setSetpoint(position.get());
//        });
//    }

//    public Command resetPosition() {
//        return runOnce(() -> {
//            wristIO.resetPosition();
//        });
//    }

//    public Command moveWristWithVoltage(double voltage) {
//        return run(() -> wristIO.setOpenLoop(voltage));
//    }

//    public Command runStopWrist() {
//        return run(() -> {
//            wristIO.setOpenLoop(0);
//        });
//    }
//
//    public Command runStopRollers() {
//        return run(() -> {
//            rollerIO.setOpenLoop(0);
//        });
//    }

//    public Command runHomingRoutine() {
//        return run(() -> {
//            isHomingFlag = true;
//            wristIO.setOpenLoop(-2.0);
//        })
//            .until(() -> wristInputs.currentAmps > IntakeConstants.autoHomeCurrentThreshold)
//            .andThen(resetPosition())
//            .andThen(() -> {
//                isDeployedFlag = false;
//                isHomingFlag = false;
//            });
//    }

    public Command toggleWristPosFlag(boolean p_on) {
        return runOnce(() -> {
            isWristMovingDown = p_on;
        });
    }

    public Command toggleWristNegFlag(boolean p_on) {
        return runOnce(() -> {
            isWristMovingUp = p_on;
        });
    }

    public Command toggleRollerFlag() {
        return runOnce(() -> {
            isRollingFlag = !isRollingFlag;
        });
    }

    public Command toggleRollerFlag(boolean on) {
        return runOnce(() -> {
            isRollingFlag = on;
        });
    }

    public Command toggleRollerDirection(boolean on) {
        return runOnce(() -> {
            isDirectionReversed = on;
        });
    }

    @Override
    public void periodic() {
//        if (!isHomingFlag) {
//            if (isDeployedFlag) {
//                setPose(IntakeWristPose.DEPLOYED);
//            } else {
//                setPose(IntakeWristPose.STOWED);
//            }
//        }
        if (isWristMovingDown && isWristMovingUp) {
            wristIO.setOpenLoop(Volts.zero());
        } else if (isWristMovingDown) {
            wristIO.setOpenLoop(Volts.of(5));
        } else if (isWristMovingUp) {
            wristIO.setOpenLoop(Volts.of(-5));
        } else if(DriverStation.isAutonomous()){
            wristIO.setOpenLoop(Volts.of(0.5));
        } else {
            wristIO.setOpenLoop(Volts.zero());
        }

        if (isRollingFlag && !isWristMovingDown) {
            // rollerIO.setClosedLoop(isDirectionReversed ? -speed.get() : speed.get());
            rollerIO.setOpenLoop(Volts.of(isDirectionReversed ? -voltage.get() : voltage.get()));
        } else {
            rollerIO.setOpenLoop(Volts.zero());
        }

        if(DriverStation.isAutonomous() && isAutoPulsing) {
            double time = Timer.getFPGATimestamp();
            double upTime = 0.5;
            double downTime = 0.5;
            time/= (upTime + downTime);
            upTime/= (upTime + downTime);
            if(time - Math.floor(time) < upTime) {
                wristIO.setOpenLoop(Volts.of(-5));
            } else {
                wristIO.setOpenLoop(Volts.of(5));
            }
        }

        Logger.recordOutput("Intake/Wrist/Deployed", isDeployedFlag);
        Logger.recordOutput("Intake/Wrist/Homing", isHomingFlag);
        Logger.recordOutput("Intake/Rollers/Running", isRollingFlag);
        Logger.recordOutput("Intake/Rollers/Reversed", isDirectionReversed);

        Logger.recordOutput("Intake/Rollers/IsMovingPos", isWristMovingDown);
        Logger.recordOutput("Intake/Rollers/IsMovingNeg", isWristMovingUp);

        Logger.recordOutput("Intake/Wrist/SetpointAdjust", pivotSetpointAdjust);

        rollerIO.periodic();
        wristIO.periodic();

        wristIO.updateInputs(wristInputs);
        rollerIO.updateInputs(rollerInputs);

        Logger.processInputs("Intake/Wrist", wristInputs);
        Logger.processInputs("Intake/Roller", rollerInputs);
        
        if (DriverStation.isDisabled()) {
            isRollingFlag = false;
        }
    }
}
