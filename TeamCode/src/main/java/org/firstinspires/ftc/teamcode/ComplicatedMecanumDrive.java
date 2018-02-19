package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotV2.RobotV2;

/**
 * Created by dmill on 2/4/2018.
 */

public class ComplicatedMecanumDrive implements IDrive {

    private final FtcGamePad driverGamepad;
    private final RobotV2 robot;

    private static final double MIN_SPEED = 0.2;

    boolean reverse = false;

    @Override
    public boolean getIsReverse() {
        return reverse;
    }

    @Override
    public void setIsReverse(boolean value) {
        reverse = value;
    }

    public ComplicatedMecanumDrive(RobotV2 robot, FtcGamePad driverGamepad) {
        this.driverGamepad = driverGamepad;
        this.robot = robot;
    }


    @Override
    public void handle() {

        double leftX = driverGamepad.getLeftStickX();
        double leftY = -driverGamepad.getLeftStickY();
        double rightX = driverGamepad.getRightStickX();

        drive(leftY, leftX, rightX);


    }

    @Override
    public void drive(double ly, double lx, double rx) {
        double r = Math.hypot(lx, ly);

        double robotAngle = Math.atan2(ly, lx) - Math.PI / 4;

        final double v1 = r * Math.cos(robotAngle) + rx;
        final double v2 = r * Math.sin(robotAngle) - rx;
        final double v3 = r * Math.sin(robotAngle) + rx;
        final double v4 = r * Math.cos(robotAngle) - rx;

        robot.Drive(v1, v2, v3, v4);
    }
}
