package org.firstinspires.ftc.teamcode.RobotV2;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.FtcGamePad;
import org.firstinspires.ftc.teamcode.IDrive;

/**
 * Created by dmill on 2/4/2018.
 */

public class ComplicatedMecanumDrive_B implements IDrive {

    private final FtcGamePad driverGamepad;
    private final RobotV2 robot;

    private static final double MIN_SPEED = 0.1;

    private DriveModes driveMode = DriveModes.FIELD;
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

    public enum DriveModes
    {
        FIELD ("Field oriented"),
        ROBOT ("Robot oriented");

        private final String name;
        DriveModes(String s) {
            name = s;
        }

        public boolean equalsName(String otherName) {
            return name.equals(otherName);
        }
        public String toString() {
            return  this.name;
        }
    }

    public DriveModes getDriveMode() {
        return  this.driveMode;
    }
    public void setDriveMode(DriveModes driveMode) {
        this.driveMode = driveMode;
    }

    public void toggleDriveMode(){
        this.driveMode = (this.driveMode == DriveModes.FIELD) ? DriveModes.ROBOT : DriveModes.FIELD;
    }

    @Override
    public void handle() {

        if(this.driveMode == DriveModes.FIELD) {
            handle_fieldOrientedDrive();
        } else {
            handle_robotOrientedDrive();
        }

    }

    @Override
    public void drive(double ly, double lx, double rx) {
        if(this.driveMode == DriveModes.FIELD) {
            fieldOrientedDrive(ly, lx, rx);
        } else {
            robotOrientedDrive(ly, lx, rx);
        }
    }

    private void fieldOrientedDrive(double forward, double right, double clockwise) {
        Orientation angles;
        angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);


        if(Math.abs(forward) < MIN_SPEED) { forward = 0; }
        if(Math.abs(right) < MIN_SPEED) { right = 0; }
        if(Math.abs(clockwise) < MIN_SPEED) { clockwise = 0; }

        double theta = angles.firstAngle;

        double temp = forward*Math.cos(theta) - right*Math.sin(theta);
        right = forward*Math.sin(theta) + right*Math.cos(theta);
        forward = temp;


        double front_left = forward + clockwise + right;
        double front_right = forward - clockwise - right;
        double rear_left = forward + clockwise - right;
        double rear_right = forward - clockwise + right;

        double max = Math.abs(front_left);
        if(Math.abs(front_right) > max) max = Math.abs(front_right);
        if(Math.abs(rear_left) > max) max = Math.abs(rear_left);
        if(Math.abs(rear_right) > max) max = Math.abs(rear_right);

        if(max > 1) {
            front_left/=max;
            front_right/=max;
            rear_left/=max;
            rear_right/=max;
        }
        robot.Drive(front_left, front_right, rear_left, rear_right);
    }
    private void robotOrientedDrive(double leftY, double leftX, double rightX) {
        double r = Math.hypot(leftX, leftY);

        double robotAngle = Math.atan2(leftY, leftX) - Math.PI / 4;

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.Drive(v1, v2, v3, v4);
    }

    public void handle_robotOrientedDrive() {

        double leftX = driverGamepad.getLeftStickX();
        double leftY = driverGamepad.getLeftStickY();


        if(reverse) {
            leftX *= -1;
            leftY *= -1;
        }

        double rightX = driverGamepad.getRightStickX();



    }



    public void handle_fieldOrientedDrive() {

        double forward = driverGamepad.getLeftStickY();
        double right = driverGamepad.getLeftStickX();
        double clockwise = driverGamepad.getRightStickX();

        fieldOrientedDrive(forward, right, clockwise);

    }


}
