package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LEDControllerIO {
    @AutoLog
    public class LEDControllerIOInputs {

    }

    public void clearAnimation();

    public void setAnimation();

    public void setLEDs(int r, int g, int b);
}
