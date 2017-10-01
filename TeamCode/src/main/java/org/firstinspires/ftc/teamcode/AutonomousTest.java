package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Thread.sleep;

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

        stopDriving();

        waitForStart();
        runtime.reset();

        driveForwardForTime(0.2, 200);
        sleep(3000);
        driveForwardForTime(-0.2, 200);
        sleep(3000);
        turnRight(0.3);
        sleep(2000);
        driveForwardForTime(0.5, 200);
        stopDriving();
        sleep(3000);
        turnLeft(0.3);
        driveForwardForTime(0.5, 50);
        sleep(2000);
        driveForwardForTime(-0.5, 100);
        turnRight(0.9);
        driveForwardForTime(0.5, 300);
        stopDriving();
        driveForwardForTime(-0.5, 300);
        turnLeft(0.1);
        driveForwardForTime(0.5, 700);
        stopDriving();

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
