package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.MotorConfiguration;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

/**
 * Created by dmill on 10/28/2017.
 */

public class Robot {


    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;

    //public DcMotor relicArmExtender = null;

    public DcMotor motorLift = null;

    public Servo colorServo = null;
    public ColorSensor colorSensor = null;


    public Servo blockGrabberLeft = null;
    public Servo blockGrabberRight = null;

    public DigitalChannel touchSensor = null;


    private static final double LIFTSPEED = 0.75;

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
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;




    public Robot(HardwareMap hardwareMap) {
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        //relicArmExtender = hardwareMap.dcMotor.get("relicArmExtender");
        motorLift = hardwareMap.dcMotor.get("lift");



        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //relicArmExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        colorServo = hardwareMap.servo.get("colorServo");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");


        blockGrabberLeft = hardwareMap.servo.get("blockLeft");
        blockGrabberRight = hardwareMap.servo.get("blockRight");


        // get a reference to our digitalTouch object.
        touchSensor = hardwareMap.get(DigitalChannel.class, "touch");

        // set the digital channel to input.
        touchSensor.setMode(DigitalChannel.Mode.INPUT);


    }

    public void stop() {
        motorLeft.setPower(0);
        motorRight.setPower(0);
        motorLift.setPower(0);
        //relicArmExtender.setPower(0);
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



//        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        final double SCALE_FACTOR = 255;
        float hsvValues[] = {0F, 0F, 0F};

        Color.RGBToHSV((int) (colorSensor.red() * SCALE_FACTOR),
                (int) (colorSensor.green() * SCALE_FACTOR),
                (int) (colorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        float hue = hsvValues[0];

        if(hue < 10 || hue > 340) {
            return  ColorSensed.RED;
        } else if(hue > 180 && hue < 240) {
            return ColorSensed.BLUE;
        } else {
            return  ColorSensed.NONE;
        }

    }



    Pair<Double,Double> grab = new Pair<>(0.05, 0.90);
    Pair<Double,Double> loose = new Pair<>(0.15, 0.80);
    Pair<Double,Double> open = new Pair<>(0.30, 0.60);
    Pair<Double,Double> start = new Pair<>(0.65, 0.25);

    private void setGrabberPosition(Pair<Double,Double> pair) {
        blockGrabberRight.setPosition(pair.getRight());
        blockGrabberLeft.setPosition(pair.getLeft());
    }

    public void GrabberOpen() {

        setGrabberPosition(open);

    }

    public void GrabberLoose() {

        setGrabberPosition(loose);

    }
    public void GrabberGrab() {

        setGrabberPosition(grab);

    }

    public void GrabberStart() {
        setGrabberPosition(start);
    }


    public void handleLiftMotor(Gamepad gamepad) {

        if(gamepad.a) {
            GrabberLiftLower();
        } else if(gamepad.y){
            GrabberLiftRaise();
        } else {
            GrabberLiftStop();
        }

    }

    public void GrabberLiftRaise() {
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setPower(LIFTSPEED);
    }

    public void GrabberLiftLower() {
        // if the digital channel returns true it's HIGH and the button is unpressed.
        if(touchSensor.getState() == true) {
            motorLift.setPower(-LIFTSPEED);
        } else {
            GrabberLiftStop();
        }
    }
    public void GrabberLiftStop() {
        motorLift.setPower(0);
    }

}
