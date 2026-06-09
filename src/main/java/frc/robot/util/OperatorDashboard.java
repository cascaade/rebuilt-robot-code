package frc.robot.util;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import lombok.Getter;

/**
 * A class that represents the driver's Elastic dashboard
 */
public class OperatorDashboard {
    /**
     *
     * @return the field object displayed on the operator dashboard
     */
    @Getter
    private static final Field2d field = new Field2d();

    private static final StringPublisher autoPathPublisher;
    private static final StringSubscriber autoPathSubscriber;
    private static final SendableChooser<String> autoMode = new SendableChooser<>();

    /**
     *
     * @return the name of the starting position of the robot
     */
    public static String getSelectedAutoStartLocation() {
        return autoMode.getSelected();
    }

    /**
     *
     * @return the string containing the segment numbers that construct the auto path
     */
    public static String getRequestedAutoPath() {
        return autoPathSubscriber.get().trim();
    }

    static {
        SmartDashboard.putData("Odometry/Field", field);

        autoMode.setDefaultOption("Auto 1", "Auto1");
        autoMode.addOption("Auto 2", "Auto2");
        autoMode.addOption("Auto 3", "Auto3");
        SmartDashboard.putData("Auto", autoMode);

        var topic = NetworkTableInstance.getDefault().getStringTopic("/SmartDashboard/Auto Path");
        autoPathPublisher = topic.publish();
        autoPathPublisher.set("1,2,9,4");
        autoPathSubscriber = topic.subscribe("1,2,9,4");
    }
}
