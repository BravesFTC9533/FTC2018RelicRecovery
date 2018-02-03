package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


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

    public Servo blockPush = null;

    public DigitalChannel touchSensor = null;



    public Servo relicGrabberServo = null;
    public Servo relicRaiserServo = null;


    public BNO055IMU imu = null;

    public static final double LIFTSPEED = 1.0;

    public enum ColorSensed {
        RED,
        BLUE,
        NONE
    }


    private static final double COLOR_RETRACTED_POSITION = 0;
    private static final double COLOR_EXTENDED_POSITION = 0.68;

    private static final int LIFT_MOTOR_MAX_POSITION = 5000;
    private static final int LIFT_MOTOR_TOLERANCE = 400;

    private static final double PUSHER_RETRACTED_POSITION = 1;
    private static final double PUSHER_EXTENDED_POSITION = 0.15;

    public static final double     REV_COUNTS_PER_MOTOR_REV = 288;     // eg: Rev Side motor
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.543 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH = (REV_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 1;
    static final double     TURN_SPEED              = 0.5;

    public static final double REV_COUNTS_PER_INCH  = (REV_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    public static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    public static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    public static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    public Robot(HardwareMap hardwareMap) {
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        //relicArmExtender = hardwareMap.dcMotor.get("relicArmExtender");
        motorLift = hardwareMap.dcMotor.get("lift");

        motorRight.setDirection(DcMotor.Direction.REVERSE);


        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //relicArmExtender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        colorServo = hardwareMap.servo.get("colorServo");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");


        blockGrabberLeft = hardwareMap.servo.get("blockLeft");
        blockGrabberRight = hardwareMap.servo.get("blockRight");

        blockPush = hardwareMap.servo.get("blockPush");

        // get a reference to our digitalTouch object.
        touchSensor = hardwareMap.get(DigitalChannel.class, "touch");

        // set the digital channel to input.
        touchSensor.setMode(DigitalChannel.Mode.INPUT);


        relicGrabberServo = hardwareMap.get(Servo.class, "relicGrabber");
        relicRaiserServo = hardwareMap.get(Servo.class, "relicRaiser");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void startIntegrator(){
        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
    }

    public void stop() {
        motorLeft.setPower(0);
        motorRight.setPower(0);
        //motorLift.setPower(0);
        //relicArmExtender.setPower(0);
    }

    public boolean isBusy() {
        return motorLeft.isBusy() || motorRight.isBusy();
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

    public void setLiftPosition(double inches) {

        int newTarget = (int)(inches * REV_COUNTS_PER_INCH);
        motorLift.setTargetPosition(newTarget);

        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

    public void pushBlockOpen() {
        blockPush.setPosition(PUSHER_EXTENDED_POSITION);
    }

    public void pushBlockClose(){
        blockPush.setPosition(PUSHER_RETRACTED_POSITION);
    }

    public void waitForTick(long periodMs) {

        sleep(periodMs);
        // sleep for the remaining portion of the regular cycle period.
//        if ( opModeIsActive()) {
//            try {
//                Thread.sleep(periodMs);
//            } catch (InterruptedException e) {
//                Thread.currentThread().interrupt();
//            }
//        }

        // Reset the cycle clock for the next pass.
        //runtime.reset();
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



    Pair<Double,Double> grab = new Pair<>(0.0, 1.0);
    Pair<Double,Double> loose = new Pair<>(0.17, 0.63);
    Pair<Double,Double> open = new Pair<>(0.30, 0.50);
    Pair<Double,Double> start = new Pair<>(1.0, 0.0);

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
            //GrabberLiftStop();

            int pos = motorLift.getCurrentPosition();

            if(pos < 200) {
                GrabberLiftStop();
            } else {
                motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorLift.setTargetPosition(pos);
                motorLift.setPower(1.0);
            }
        }

    }


    public void GrabberLiftRaise() {

        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLift.setPower(LIFTSPEED);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(motorLift.getCurrentPosition() >= LIFT_MOTOR_MAX_POSITION - LIFT_MOTOR_TOLERANCE) {
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setTargetPosition(LIFT_MOTOR_MAX_POSITION);
        } else {
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void GrabberLiftLower() {


        motorLift.setPower(-LIFTSPEED);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       // boolean pressed = touchSensor.getState() != true;

        // if the digital channel returns true it's HIGH and the button is unpressed.
        if( motorLift.getCurrentPosition() > LIFT_MOTOR_TOLERANCE) {

            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {

            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setTargetPosition(0);

            //GrabberLiftStop();
        }
    }

    public void GrabberLiftStop() {
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setPower(0);
    }


    private boolean relicLifted= false;

    public void toggleRelicLift() {
        if(relicLifted) {
            lowerRelic();
        } else {
            liftRelic();
        }
        relicLifted = !relicLifted;
    }
    public void liftRelic() {

        relicRaiserServo.setPosition(0.0);
    }

    public void lowerRelic() {
        relicRaiserServo.setPosition(1.0);
    }




    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
