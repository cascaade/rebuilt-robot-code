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
    public boolean isRollingFlag = true;
    private boolean isDirectionReversed = false;

    private boolean isWristMovingUp = false;
    private boolean isWristMovingDown = false;
    private boolean isAutoPulsing = false;
    private int intakePulses = 0;
    private boolean pulsingUp = false;
    private boolean pulsingDown = false;
    private double pulseTimer = 0;
    private boolean upPressed = false;
    private boolean downPressed = false;

    private Angle pivotSetpointAdjust = Radians.mutable(0.0);

    private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    LoggedNetworkNumber speed = new LoggedNetworkNumber("Intake/Roller/Speed", IntakeConstants.intakeMaxSpeed);
    LoggedNetworkNumber voltage = new LoggedNetworkNumber("Intake/Roller/Voltage", 12);
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
            intakePulses = 0;
            pulsingDown = false;
            pulsingUp = false;
            isWristMovingDown = p_on;
            downPressed = p_on;
        });
    }

    public Command toggleWristNegFlag(boolean p_on) {
        return runOnce(() -> {
            intakePulses = 0;
            pulsingDown = false;
            pulsingUp = false;
            isWristMovingUp = p_on;
            upPressed = p_on;
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

    public Command addPulse() {
        return runOnce(() -> {
            if(!upPressed && !downPressed) intakePulses++;
            else intakePulses = 0;
        });
    }

    @Override
    public void periodic() {
        if (DriverStation.isAutonomous()) toggleRollerFlag(true);
//        if (!isHomingFlag) {
//            if (isDeployedFlag) {
//                setPose(IntakeWristPose.DEPLOYED);
//            } else {
//                setPose(IntakeWristPose.STOWED);
//            }
//        }

        
        //check if any pulses are in queue
        if(intakePulses > 0) {
            double time = Timer.getFPGATimestamp();
            //check if the pulse has been initiated, if not, it starts up motion
            if(!pulsingUp && !pulsingDown) {
                pulseTimer = time;
                pulsingUp = true;
                pulsingDown = false;
                isWristMovingUp = true;
                isWristMovingDown = false;
            } else if(pulsingUp) {
                // if moving up, checks time, if done, it starts down motion
                if(time - pulseTimer > 0.4) {
                    pulseTimer = time;
                    pulsingUp = false;
                    pulsingDown = true;
                    isWristMovingUp = false;
                    isWristMovingDown = true;
                }
            } else if (pulsingDown) {
                //if moving down, checks time, if done, it terminates pulse
                if(time - pulseTimer > 0.4) {
                    pulseTimer = 0;
                    pulsingUp = false;
                    pulsingDown = false;
                    isWristMovingUp = false;
                    isWristMovingDown = false;
                    intakePulses--;
                }
            }
        }
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
            isRollingFlag = true;
        }
    }
}

