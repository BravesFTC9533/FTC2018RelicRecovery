package org.firstinspires.ftc.teamcode.RobotV2;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.FtcGamePad;
import org.firstinspires.ftc.teamcode.IDrive;
import org.firstinspires.ftc.teamcode.SensorBNO055IMU;

/**
 * Created by dmill on 2/4/2018.
 */

public class ComplicatedMecanumDrive_B implements IDrive {

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

    public ComplicatedMecanumDrive_B(RobotV2 robot, FtcGamePad driverGamepad) {
        this.driverGamepad = driverGamepad;
        this.robot = robot;
    }


    @Override
    public void handle() {

        double leftX = driverGamepad.getLeftStickX();
        double leftY = -driverGamepad.getLeftStickY();
        double rightX = driverGamepad.getRightStickX();

        double r = Math.hypot(leftX, leftY);

        double robotAngle = Math.atan2(leftY, leftX) - Math.PI / 4;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.Drive(v1, v2, v3, v4);


    }

    public void fieldDrive(){
        double forward = driverGamepad.getLeftStickY();
        double strafe = driverGamepad.getLeftStickX();
        double rotation = driverGamepad.getRightStickX();

        Orientation angles;

        angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double gyroDegrees = angles.firstAngle;
        double gyroRadians = gyroDegrees * (Math.PI/180);

        double r = Math.hypot(forward, strafe);

        double temp = forward * Math.cos(gyroRadians) + strafe * Math.sin(gyroRadians);
        strafe = -forward * Math.sin(gyroRadians) + strafe * Math.cos(gyroRadians);
        forward = temp;

        final double v1 = r * Math.cos(gyroRadians) + rotation;
        final double v2 = r * Math.sin(gyroRadians) - rotation;
        final double v3 = r * Math.sin(gyroRadians) + rotation;
        final double v4 = r * Math.cos(gyroRadians) - rotation;

        robot.Drive(v1, v2, v3, v4);

    }
}
