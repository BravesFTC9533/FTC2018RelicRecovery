package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.RevColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ConceptVuMarkIdentification;

import static java.lang.Thread.sleep;

//Created by 9533 on 9/30/2017


@Autonomous(name = "AutonomousRED", group = "Tests")
public class AutonomousRED extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();

    private DcMotor motorRight;
    private DcMotor motorLeft;

    private Servo colorServo;

    ColorSensor colorSensor;
    boolean isRed = false;

    ConceptVuMarkIdentification vuMarkIdentification;

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
        String shelfLocation = vuMarkIdentification.vuforia.toString();
        if(shelfLocation.equals("CENTER")){
            colorServo.setPosition(COLOR_EXTENDED_POSITION);
            colorServo.setPosition(COLOR_RETRACTED_POSITION);
        }
        else {
            return;
        }
        colorServo.setPosition(COLOR_EXTENDED_POSITION);
        sleep(2000);
        telemetry.addData("Alpha", colorSensor.alpha());
        telemetry.addData("Red  ", colorSensor.red());
        telemetry.addData("Green", colorSensor.green());
        telemetry.addData("Blue ", colorSensor.blue());
        if(colorSensor.red() > 45){
            isRed = true;
        } else {
            isRed = false;
        }
        if(isRed){
            driveForwardForTime(1, 50);
            stopDriving();
            colorServo.setPosition(COLOR_RETRACTED_POSITION);
        }else {
            driveForwardForTime(-1, 50);
            stopDriving();
            colorServo.setPosition(COLOR_RETRACTED_POSITION);
        }
        //colorServo.setPosition(COLOR_RETRACTED_POSITION);

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
