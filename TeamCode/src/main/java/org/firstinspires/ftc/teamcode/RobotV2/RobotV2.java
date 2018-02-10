package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

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
//        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
//        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        //SetMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public DcMotor.RunMode GetMode() {
        return motorFrontLeft.getMode();
    }

    public void SetMode(DcMotor.RunMode runMode) {
        motorFrontLeft.setMode(runMode);
        motorFrontRight.setMode(runMode);
//        motorBackLeft.setMode(runMode);
//        motorBackRight.setMode(runMode);
    }

    public void SetPIDCoefficients(DcMotor.RunMode runMode, double new_p, double new_i, double new_d) {
        PIDCoefficients new_pid = new PIDCoefficients(new_p, new_i, new_d);

        ((DcMotorEx)motorFrontLeft).setPIDCoefficients(runMode, new_pid);
        ((DcMotorEx)motorFrontRight).setPIDCoefficients(runMode, new_pid);
//        ((DcMotorEx)motorBackLeft).setPIDCoefficients(runMode, new_pid);
//        ((DcMotorEx)motorBackRight).setPIDCoefficients(runMode, new_pid);

    }
    public PIDCoefficients GetPIDCoefficients(DcMotor.RunMode runMode) {
        return  ((DcMotorEx)motorFrontLeft).getPIDCoefficients(runMode);
    }

    public void Drive(double fl, double fr, double bl, double br) {

        motorFrontLeft.setPower(fl);
        motorFrontRight.setPower(fr);
//        motorBackLeft.setPower(bl);
//        motorBackRight.setPower(br);

    }

    public void stop()
    {
        Drive(0,0,0,0);
    }
}