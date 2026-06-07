package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import lombok.Getter;

public class OperatorDashboard {
    @Getter
    private static final Field2d field = new Field2d();

    static {
        SmartDashboard.putData("Odometry/Field", field);
    }
}
