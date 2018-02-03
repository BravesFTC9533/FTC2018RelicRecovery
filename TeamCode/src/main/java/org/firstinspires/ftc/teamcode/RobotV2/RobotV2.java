package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by 9533 on 2/3/2018.
 */

public class RobotV2 {

    public DcMotor motorFrontLeft = null;

    public RobotV2(HardwareMap hardwareMap) {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveForward(){
        motorFrontLeft.setPower(1);
    }

    public void stop(){
        motorFrontLeft.setPower(0);
    }
}