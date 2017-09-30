package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 9533 on 9/30/2017.
 */

@Autonomous(name = "AutonomousTest", group = "Tests")
public class AutonomousTest extends LinearOpMode{
    ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorRight;
    private DcMotor motorLeft;

    public void runOpMode() throws InterruptedException {
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");

        waitForStart();
        runtime.reset();

        driveForwardForTime(0.5, 500);
        sleep(3000);
        driveForwardForTime(-0.5, 500);
        sleep(3000);
        turnRight(0.5);
        driveForwardForTime(0.5, 1500);
        stopDriving();
        turnLeft(0.4);

    }

    void driveForwardForTime(double power, long time){
        motorRight.setPower(power);
        motorLeft.setPower(power);
        sleep(time);
        stopDriving();
    }

    void turnRight(double power){
        motorRight.setPower(-power);
        motorLeft.setPower(power);
        sleep(550);
        stopDriving();
    }

    void turnLeft(double power){
        motorRight.setPower(power);
        motorLeft.setPower(-power);
        sleep(550);
        stopDriving();
    }

    void stopDriving(){
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }
}
