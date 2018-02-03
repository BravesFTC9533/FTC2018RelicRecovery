package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.RobotV2.RobotV2;

/**
 * Created by 9533 on 2/3/2018.
 */

public class MecanumDrive implements IDrive {

    private final FtcGamePad driverGamepad;
    private final RobotV2 robot;

    boolean reverse = false;

    MecanumDrive(RobotV2 robot, FtcGamePad driveGamepad){
        this.driverGamepad = driveGamepad;
        this.robot = robot;
    }

    public boolean getIsReverse(){
        return reverse;
    }

    public void setIsReverse(boolean value){
        reverse = value;
    }

    public void handle(){
        robot.driveForward();
    }
}
