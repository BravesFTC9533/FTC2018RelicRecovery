package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Brave-inators Team #9533 on 9/21/2017.
 * Go Braves!!
 */

@TeleOp(name = "TeleOp Tutorial", group = "Tutorials")
public class TeleOpTutorial extends LinearOpMode{

    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor relicArmExtender;
    private Servo armServo;
    double motorPower = 50;

    @Override
    public void runOpMode () throws InterruptedException{

        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        relicArmExtender = hardwareMap.dcMotor.get("relicArmExtender");

        //armServo = hardwareMap.servo.get("armServo");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            motorLeft.setPower(-gamepad1.left_stick_y * 0.5);
            motorRight.setPower(-gamepad1.right_stick_y * 0.5);
            relicArmExtender.setPower(gamepad2.left_trigger * 0.5);
            relicArmExtender.setPower(-gamepad2.right_trigger * 0.5);
            idle();
        }
    }
}
