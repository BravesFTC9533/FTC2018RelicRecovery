package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.MotorConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

/**
 * Created by dmill on 10/28/2017.
 */

public class Robot {


    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;



    public Servo colorServo = null;
    public ColorSensor colorSensor = null;


    public Servo blockGrabberLeft = null;
    public Servo blockGrabberRight = null;


    public enum ColorSensed {
        RED,
        BLUE,
        NONE
    }


    private static final int COLOR_SENSOR_RED_THRESHOLD = 50;
    private static final int COLOR_SENSOR_BLUE_THRESHOLD = 40;

    private static final double COLOR_RETRACTED_POSITION = 1;
    private static final double COLOR_EXTENDED_POSITION = 0.4;


    private static final double GRABBER_LEFT_OPEN_POSITION = 0;
    private static final double GRABBER_RIGHT_OPEN_POSITION = 0;

    private static final double GRABBER_LEFT_CLOSE_POSITION = 0;
    private static final double GRABBER_RIGHT_CLOSE_POSITION = 0;



    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;




    public Robot(HardwareMap hardwareMap) {
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        colorServo = hardwareMap.servo.get("colorServo");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");


        blockGrabberLeft = hardwareMap.servo.get("blockLeft");
        blockGrabberRight = hardwareMap.servo.get("blockRight");


    }

    public void stop() {
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    public boolean isBusy() {
        return motorLeft.isBusy() && motorRight.isBusy();
    }
    public void setPower(double left, double right) {
        motorLeft.setPower(left);
        motorRight.setPower(right);

    }
    public void  setNewPosition(double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;



        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        newLeftTarget = motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        motorLeft.setTargetPosition(newLeftTarget);
        motorRight.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setMode(DcMotor.RunMode mode) {
        motorLeft.setMode(mode);
        motorRight.setMode(mode);
    }


    public void extendColorArm() {
        colorServo.setPosition(COLOR_EXTENDED_POSITION);

    }

    public void retractColorArm() {
        colorServo.setPosition(COLOR_RETRACTED_POSITION);

    }


    public ColorSensed SenseJewel() {
        if(colorSensor.red() > COLOR_SENSOR_RED_THRESHOLD) {
            return  ColorSensed.RED;
        } else if (colorSensor.blue() > COLOR_SENSOR_BLUE_THRESHOLD ){
            return ColorSensed.BLUE;
        } else {
            return ColorSensed.NONE;
        }
    }



    public void GrabberOpen() {

    }

    public void GrabberClose() {

    }

    public void GrabberRaise() {

    }

    public void GrabberLower() {

    }

}
