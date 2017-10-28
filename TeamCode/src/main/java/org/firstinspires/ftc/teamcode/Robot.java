package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by dmill on 10/28/2017.
 */

public class Robot {


    public DcMotor motorLeft = null;
    public DcMotor motorRight = null;

    public Servo colorServo = null;
    public ColorSensor colorSensor = null;


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


        colorServo = hardwareMap.servo.get("colorServo");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");


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
    public void setNewPosition(double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        newLeftTarget = motorLeft.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = motorRight.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        motorLeft.setTargetPosition(newLeftTarget);
        motorRight.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMode(DcMotor.RunMode mode) {
        motorLeft.setMode(mode);
        motorRight.setMode(mode);
    }
}
