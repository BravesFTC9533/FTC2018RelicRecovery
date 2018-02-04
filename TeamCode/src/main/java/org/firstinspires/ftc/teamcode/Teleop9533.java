package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

/**
 * Created by dmill on 10/29/2017.
 */

@TeleOp(name = "TeleOp", group = "Competition")
public class Teleop9533 extends LinearOpMode9533 implements FtcGamePad.ButtonHandler {


    //private FtcGamePad driverGamepad;
    //private FtcGamePad operatorGamepad;
    //private Robot robot;

    //private IDrive robotDrive;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;


    int lastPositionLeft = 0;
    int lastPositionRight = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);
        driverGamepad = new FtcGamePad("DriverGamepad", gamepad1, this);
        operatorGamepad = new FtcGamePad("OperatorGamepad", gamepad2, this);

        robotDrive = new GTADrive2(robot, driverGamepad);


        telemetry.addData("Waiting for start..", "");
        telemetry.update();



        // Set up our telemetry dashboard
        composeTelemetry();

        waitForStart();

        robot.retractColorArm();
        lastPositionLeft = robot.motorLeft.getCurrentPosition();
        lastPositionRight = robot.motorRight.getCurrentPosition();


        while(opModeIsActive()){

            driverGamepad.update();
            operatorGamepad.update();

            if(!parking) {
                robotDrive.handle();
            }

            robot.handleLiftMotor(gamepad2);


            telemetry.update();
        }

        robot.stop();



    }


    @Override
    public void gamepadButtonEvent(FtcGamePad gamepad, int button, boolean pressed) {

        if(gamepad == driverGamepad) {
            handleDriverGamepad(gamepad, button, pressed);
        } else if(gamepad == operatorGamepad) {
            handleOperatorGamepad(gamepad, button, pressed);
        }
    }

    private void handleDriverGamepad(FtcGamePad gamepad, int button, boolean pressed) {

        //handle buttons here.
        switch (button)
        {
            case FtcGamePad.GAMEPAD_A: //counter clockwise
                if(pressed) {

                    turn90slow(Autonomous9533.TurnDirection.CLOCKWISE);
                }
                break;

            case FtcGamePad.GAMEPAD_B: //clockwise
                if(pressed) {

                    turn90slow(Autonomous9533.TurnDirection.COUNTERCLOCKWISE);
                }
                break;

            case FtcGamePad.GAMEPAD_X:
                if(pressed){
                    lastPositionLeft = 0;
                    lastPositionRight = 0;
                    robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sleep(1000);
                    robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                }
                break;

            case FtcGamePad.GAMEPAD_Y:
                if(pressed){
                    lastPositionRight = robot.motorRight.getCurrentPosition();
                    lastPositionLeft = robot.motorLeft.getCurrentPosition();

                    robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                break;

            case FtcGamePad.GAMEPAD_LBUMPER:
                //drivePowerScale = pressed? 0.5: 1.0;
                break;

            case FtcGamePad.GAMEPAD_RBUMPER:

                if(pressed) {
                    robotDrive.setIsReverse(!robotDrive.getIsReverse());
                }

                break;
            case FtcGamePad.GAMEPAD_START:
                if(pressed) {
                    parkRobot();
                }
                break;
            case FtcGamePad.GAMEPAD_BACK:

                break;

        }


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

    private void handleOperatorGamepad(FtcGamePad gamepad, int button, boolean pressed) {


        switch (button)
        {
            case FtcGamePad.GAMEPAD_A:
                // used for cryptoblock lift
                break;
            case FtcGamePad.GAMEPAD_B:
                if(pressed){
                    robot.pushBlockOpen();
                } else {
                    robot.pushBlockClose();
                }
                break;
            case FtcGamePad.GAMEPAD_X:


                if(pressed) {
                    robot.toggleRelicLift();
                }

                break;
            case FtcGamePad.GAMEPAD_Y:
                // used for cryptoblock lift
                break;

            case FtcGamePad.GAMEPAD_LBUMPER:
                if(pressed) {
                     robot.GrabberOpen();
                }
                break;

            case FtcGamePad.GAMEPAD_RBUMPER:
                if(pressed){
                    robot.GrabberGrab();
                } else {
                    robot.GrabberLoose();
                }

                break;
        }

    }


    boolean parking = false;
    void parkRobot() {
        parking = true;
        robot.setPower(1, 1);
        sleep(250);
        robot.setPower(-1, -1);
        sleep(150);
        robot.setPower(-0.4, -0.4);
        sleep(750);
        robot.setPower(0,0);
        ElapsedTime runtime = new ElapsedTime();

        while(opModeIsActive() && runtime.seconds() < 10) {
            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double degrees = AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle);

            if(degrees > -2 && degrees < 2) {
                break;
            }

            double distance;
            if(degrees < 0) {
                //move backwards
                distance = -2;
            } else {
                //move forwards
                distance = 1;
            }

            encoderDrive(0.7, distance, distance, 4, true);
        }

        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        parking = false;
    }




    /*
    *  Method to perform a relative move, based on encoder counts.
    *  Encoders are not reset as the move is based on the current position.
    *  Move will stop if any of three conditions occur:
    *  1) Move gets to the desired position
    *  2) Move runs out of time
    *  3) Driver stops the opmode running.
    */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, boolean holdPosition) {


        int targetLeft, targetRight, currentRight, currentLeft;
        int differenceLeft, differenceRight;

        boolean maxed = false;
        ElapsedTime runtime = new ElapsedTime();

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.setNewPosition(leftInches, rightInches);

            currentLeft = robot.motorLeft.getCurrentPosition();
            int newLeftTarget = currentLeft + (int)(leftInches * robot.COUNTS_PER_INCH);

            int scale = newLeftTarget - currentLeft;


            // reset the timeout time and start motion.
            runtime.reset();

            double currentSpeed = 0;
            double multiplier = 0;


            robot.setPower(currentSpeed , currentSpeed);


            int lastPosition = 0;
            double lastRuntime = 0;

            int slowdownTick = 150;
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.isBusy())) {
                targetLeft = robot.motorLeft.getTargetPosition();
                targetRight =robot.motorRight.getTargetPosition();
                currentLeft = robot.motorLeft.getCurrentPosition();
                currentRight = robot.motorRight.getCurrentPosition();
                differenceLeft = Math.abs(Math.abs(targetLeft) - Math.abs(currentLeft));

                if(maxed) {
                    double newSpeed = Easing.Interpolate(1 - (differenceLeft / scale), Easing.Functions.QuinticEaseIn);
                    currentSpeed = newSpeed;
                    if(currentSpeed < 0.2) {
                        currentSpeed = 0.2;
                    }
                }
                else if(currentSpeed < speed) {
                    multiplier = Easing.Interpolate(runtime.seconds() * 4, Easing.Functions.CubicEaseOut);
                    currentSpeed = speed * multiplier;
                }


                telemetry.addLine()
                        .addData("Multiplier", "%7f", multiplier)
                        .addData("Speed", "%7f", currentSpeed);

                //telemetry.update();

                if(currentSpeed >= speed) {
                    currentSpeed = speed;
                    maxed = true;
                }
                robot.setPower(currentSpeed, currentSpeed);


                // Display it for the driver.
                telemetry.addLine().addData("Target",  "Running to %7d :%7d",
                        targetLeft,
                        targetRight);
                telemetry.addLine().addData("Current",  "Running at %7d :%7d",
                        currentLeft,
                        currentRight);
                telemetry.update();
            }

            if(holdPosition==false) {
                // Stop all motion;
                robot.stop();

                // Turn off RUN_TO_POSITION
                robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }


    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
        telemetry.addLine()
                .addData("Left Grab", new Func<String>() {
                    @Override public String value() {
                        return formatDouble(robot.blockGrabberLeft.getPosition());
                    }
                })
                .addData("Right Grab", new Func<String>() {
                    @Override public String value() {
                        return formatDouble(robot.blockGrabberRight.getPosition());
                    }
                })
                .addData("Lift", new Func<String>() {
                    @Override public String value() {
                        return formatDouble(robot.motorLift.getCurrentPosition());
                    }
                });
        telemetry.addLine()
                .addData("Left Pos", new Func<String>() {
                    @Override public String value() {
                        return formatDouble(robot.motorLeft.getCurrentPosition() - lastPositionLeft);
                    }
                })
                .addData("Right Pos", new Func<String>() {
                    @Override public String value() {
                        return formatDouble(robot.motorRight.getCurrentPosition() - lastPositionRight);
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }


    String formatDouble(double value) {
        return  String.format("%.1f", value);
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
