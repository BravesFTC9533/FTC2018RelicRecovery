package org.firstinspires.ftc.teamcode.RobotV2;

import android.text.method.Touch;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Quad;

import java.util.ArrayList;


/**
 * Created by 9533 on 2/3/2018.
 */

public class RobotV2 {

    public DcMotorEx motorFrontLeft = null;
    public DcMotorEx motorFrontRight = null;
    public DcMotorEx motorBackLeft = null;
    public DcMotorEx motorBackRight = null;

    public DcMotor motorLift = null;

    public Servo flipperServoLeft = null;
    public Servo flipperServoRight = null;


    public DigitalChannel touchSensor = null;

    public double MIN_SERVO = 0.3;
    public double LIFT_POWER = 1;

    public BNO055IMU imu = null;

    public static final double     REV_COUNTS_PER_MOTOR_REV = 288;     // eg: Rev Side motor
    public static final double     DRIVE_GEAR_REDUCTION    = 0.5;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH = (REV_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);



    public RobotV2(HardwareMap hardwareMap) {
        motorFrontLeft =  (DcMotorEx)hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = (DcMotorEx)hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft =   (DcMotorEx)hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight =  (DcMotorEx)hardwareMap.dcMotor.get("motorBackRight");


        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        motorLift = hardwareMap.get(DcMotor.class, "liftMotor");
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setDirection(DcMotorSimple.Direction.REVERSE);


        flipperServoLeft = hardwareMap.get(Servo.class, "flipperServoLeft");
        flipperServoRight = hardwareMap.get(Servo.class, "flipperServoRight");
        flipperServoRight.setDirection(Servo.Direction.REVERSE);


        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);


        //SetMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; //"BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }

    void updatePID(double p, double i, double d) {
        PIDCoefficients pidNew = new PIDCoefficients(p, i, d);

        motorFrontLeft.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motorFrontRight.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motorBackLeft.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        motorBackRight.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);

        motorFrontLeft.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);
        motorFrontRight.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);
        motorBackLeft.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);
        motorBackRight.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);

    }

    public DcMotor.RunMode GetMode() {
        return motorFrontLeft.getMode();
    }

    public void SetMode(DcMotor.RunMode runMode) {
        motorFrontLeft.setMode(runMode);
        motorFrontRight.setMode(runMode);
        motorBackLeft.setMode(runMode);
        motorBackRight.setMode(runMode);
    }


    public PIDCoefficients GetPIDCoefficients(DcMotor.RunMode runMode) {
        return  ((DcMotorEx)motorFrontLeft).getPIDCoefficients(runMode);
    }

    public void Drive(double fl, double fr, double bl, double br) {

        motorFrontLeft.setPower(fl);
        motorFrontRight.setPower(fr);
        motorBackLeft.setPower(bl);
        motorBackRight.setPower(br);

    }

    public boolean isBusy() {
        return  isBusy(true);
    }
    public boolean isBusy(boolean allMotors){
        if(allMotors) {
            return motorFrontLeft.isBusy() ||
                    motorFrontRight.isBusy() ||
                    motorBackLeft.isBusy() ||
                    motorBackRight.isBusy();
        } else {
            return motorFrontLeft.isBusy() &&
                    motorFrontRight.isBusy() &&
                    motorBackLeft.isBusy() &&
                    motorBackRight.isBusy();
        }
    }

    public ArrayList<Integer> getPositions() {
        ArrayList<Integer> list = new ArrayList<>(4);
        list.add(motorFrontLeft.getCurrentPosition());
        list.add(motorFrontRight.getCurrentPosition());
        list.add(motorBackLeft.getCurrentPosition());
        list.add(motorBackRight.getCurrentPosition());
        return  list;
    }

    public void stop()
    {
        Drive(0,0,0,0);
    }


    public Quad<Integer, Integer, Integer, Integer> getCurrentPosition() {
        return new Quad<>(motorFrontLeft.getCurrentPosition(), motorFrontRight.getCurrentPosition(), motorBackLeft.getCurrentPosition(), motorBackRight.getCurrentPosition());
    }
    public Quad<Integer, Integer, Integer, Integer> calculateNewPositions(double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches){
        int leftTargetFront = motorFrontLeft.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
        int rightTargetFront = motorFrontRight.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
        int leftTargetBack = motorBackLeft.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
        int rightTargetBack = motorBackRight.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

        return new Quad<>(leftTargetFront, rightTargetFront, leftTargetBack, rightTargetBack);
    }

    public Quad<Integer, Integer, Integer, Integer> setNewPosition(double inches) {
        return setNewPosition(inches, inches, inches, inches);
    }
    public Quad<Integer, Integer, Integer, Integer> setNewPosition(double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches) {
        Quad<Integer, Integer, Integer, Integer>target = calculateNewPositions(leftFrontInches, rightFrontInches, leftBackInches, rightBackInches);

        motorFrontLeft.setTargetPosition(target.getA());
        motorFrontRight.setTargetPosition(target.getB());

        motorBackLeft.setTargetPosition(target.getC());
        motorBackRight.setTargetPosition(target.getD());

        // Turn On RUN_TO_POSITION
        setRunToPosition();
        return target;
    }


    public Quad<Integer, Integer, Integer, Integer> setNewPositionTicks(int leftFront, int rightFront, int leftBack, int rightBack) {

        int leftTargetFront = motorFrontLeft.getCurrentPosition() + leftFront;
        int rightTargetFront = motorFrontRight.getCurrentPosition() + rightFront;
        int leftTargetBack = motorBackLeft.getCurrentPosition() + leftBack;
        int rightTargetBack = motorBackRight.getCurrentPosition() + rightBack;

        motorFrontLeft.setTargetPosition(leftTargetFront);
        motorFrontRight.setTargetPosition(rightTargetFront);

        motorBackLeft.setTargetPosition(leftTargetBack);
        motorBackRight.setTargetPosition(rightTargetBack);

        // Turn On RUN_TO_POSITION
        setRunToPosition();
        return new Quad<>(leftTargetFront, rightTargetFront, leftTargetBack, rightTargetBack);
    }


    public void setRunUsingEncoders(){
        SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setRunToPosition() {
        SetMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setRunWithoutEncoders() {
        SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void closeBlockFlipperServo() {
        servoPosition -= servoDelta;

        if(servoPosition < MIN_SERVO) {
            servoPosition = MIN_SERVO;
        }

        setBlockFlipperServoPosition(servoPosition);
    }

    private void setBlockFlipperServoPosition(double position) {
        flipperServoLeft.setPosition(position);
        flipperServoRight.setPosition(position);
    }

    static final double servoDelta = 0.02;
    public static double servoDelayDelta = 1000/100;

    public double servoPosition = MIN_SERVO;


    public void incrementBlockFlipperServo() {
        servoPosition += servoDelta;
        if(servoPosition > 1) {
            servoPosition = 1;
        }
        setBlockFlipperServoPosition(servoPosition);

    }

    public double getServoPosition() {
        return  servoPosition;
    }


    public void raiseLift() {
        motorLift.setPower(LIFT_POWER);
    }
    public void stopLift() {
        motorLift.setPower(0);
    }
    public void lowerLift() {
        motorLift.setPower(-LIFT_POWER);
    }

    public boolean touchSensorPressed() {
        return  !(touchSensor.getState() == true);
    }

}