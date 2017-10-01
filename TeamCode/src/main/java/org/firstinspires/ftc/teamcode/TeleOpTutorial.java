package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 9533 on 9/21/2017.
 */

@TeleOp(name = "TeleOp Tutorial", group = "Tutorials")
public class TeleOpTutorial extends LinearOpMode{

    private DcMotor motorLeft;
    private DcMotor motorRight;

    private Servo armServo;
    double motorPower = 50;

    @Override
    public void runOpMode () throws InterruptedException{

        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        armServo = hardwareMap.servo.get("armServo");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            motorLeft.setPower(gamepad1.left_stick_y * 0.5);
            motorRight.setPower(-gamepad1.right_stick_y * 0.5);
            idle();
        }
    }
}
