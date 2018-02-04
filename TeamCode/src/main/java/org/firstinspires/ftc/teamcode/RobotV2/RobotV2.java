package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by 9533 on 2/3/2018.
 */

public class RobotV2 {

    public DcMotor motorFrontLeft = null;
    public DcMotor motorFrontRight = null;
    public DcMotor motorBackLeft = null;
    public DcMotor motorBackRight = null;

    public RobotV2(HardwareMap hardwareMap) {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);


        //SetMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void SetMode(DcMotor.RunMode runMode) {
        motorFrontLeft.setMode(runMode);
        motorFrontRight.setMode(runMode);
        motorBackLeft.setMode(runMode);
        motorBackRight.setMode(runMode);
    }


    public void Drive(double fl, double fr, double bl, double br) {

        motorFrontLeft.setPower(fl);
        motorFrontRight.setPower(fr);
        motorBackLeft.setPower(bl);
        motorBackRight.setPower(br);

    }

    public void stop()
    {
        Drive(0,0,0,0);
    }
}