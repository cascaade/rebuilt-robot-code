package frc.robot.subsystems.led;

import org.littletonrobotics.junction.AutoLog;

public interface LEDControllerIO {
    @AutoLog
    public class LEDControllerIOInputs {
        LEDAnimation animation = null;
    }

    public void clearAnimation();

    public void setAnimation(LEDAnimation animation);

    public void setLEDs(int r, int g, int b);

    public void update();
}
