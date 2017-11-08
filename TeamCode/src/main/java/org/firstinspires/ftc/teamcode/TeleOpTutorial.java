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
    private Servo blockLeft;
    private Servo blockRight;
    double motorPower = 50;

    @Override
    public void runOpMode () throws InterruptedException{

        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        relicArmExtender = hardwareMap.dcMotor.get("relicArmExtender");
        blockLeft = hardwareMap.servo.get("blockLeft");
        blockRight = hardwareMap.servo.get("blockRight");

        //armServo = hardwareMap.servo.get("armServo");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            motorLeft.setPower(-gamepad1.left_stick_y * 0.5);
            motorRight.setPower(-gamepad1.right_stick_y * 0.5);
            relicArmExtender.setPower(gamepad2.left_trigger * 0.5);
            relicArmExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            relicArmExtender.setPower(-gamepad2.right_trigger * 0.5);

            if(gamepad2.left_bumper){//Open position
                blockLeft.setPosition(0.4);
                blockRight.setPosition(0.4);
            }
            if(gamepad2.right_bumper){//Close position
                blockLeft.setPosition(0);
               blockRight.setPosition(1);
            }
            idle();
        }
    }
}
