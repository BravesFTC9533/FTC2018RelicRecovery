package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Created by 9533 on 9/30/2017


@Autonomous(name = "AutonomousBLUE", group = "Tests")
@Disabled
public class AutonomousBLUE extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorRight;
    private DcMotor motorLeft;

    private Servo colorServo;

    ColorSensor colorSensor;

    boolean isBlue = false;
private static final double COLOR_RETRACTED_POSITION = 1;
private static final double COLOR_EXTENDED_POSITION = 0.4;

    public void runOpMode() throws InterruptedException {
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");

        colorServo = hardwareMap.servo.get("colorServo");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        //colorServo.scaleRange(0, 1);
        colorServo.setPosition(COLOR_RETRACTED_POSITION);

        stopDriving();

        waitForStart();
        runtime.reset();

        //START AUTNOMOUS BLUE
        colorServo.setPosition(COLOR_EXTENDED_POSITION);
        sleep(2000);
        telemetry.addData("Alpha", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        if(colorSensor.red() < 30){
            motorRight.setPower(1);
            sleep(100);
            stopDriving();
            colorServo.setPosition(COLOR_RETRACTED_POSITION);
            sleep(50);
            motorRight.setPower(-1);
            sleep(100);
            stopDriving();
        } else {
            motorLeft.setPower(1);
            sleep(100);
            stopDriving();
            colorServo.setPosition(COLOR_RETRACTED_POSITION);
            sleep(50);
            motorLeft.setPower(-1);
            sleep(100);
            stopDriving();
        }
    }

   void driveForwardForTime(double power, long time){
        motorRight.setPower(power);
        motorLeft.setPower(-power);
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
