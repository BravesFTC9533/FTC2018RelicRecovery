package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by dmill on 12/16/2017.
 */

public abstract class LinearOpMode9533 extends LinearOpMode {

    protected FtcGamePad driverGamepad;
    protected FtcGamePad operatorGamepad;
    protected Robot robot;

    protected IDrive robotDrive;
    protected String currentStep = "";
    protected double speed = 0.0;
    protected double fast_speed = 1.0;
    protected double turn_speed = 0.5;




    DcMotorEx motorExLeft;
    DcMotorEx motorExRight;



    public void turn90Old(Autonomous9533.TurnDirection direction) {
        double turn90Inches = (3.51 * 3.1415) * (313/robot.REV_COUNTS_PER_MOTOR_REV);

        if(direction == Autonomous9533.TurnDirection.CLOCKWISE) {
            //maneuver build for counter-clockwise, so reverse
            turn90Inches = -turn90Inches;
        }
        updateStep("Turning 90 degrees");

        encoderDrive(turn_speed, -turn90Inches, turn90Inches, 5.0);
        updateStep("Finished turning 90 degrees");
    }


    public void turn90(Autonomous9533.TurnDirection direction) {
        PIDCoefficients pidCurrent = motorExLeft.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDCoefficients pidNew = new PIDCoefficients(1.0, 0, 0);

        motorExLeft.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);
        motorExRight.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);

        double turn90Inches = (3.51 * 3.1415) * (313/robot.REV_COUNTS_PER_MOTOR_REV);

        if(direction == Autonomous9533.TurnDirection.CLOCKWISE) {
            //maneuver build for counter-clockwise, so reverse
            turn90Inches = -turn90Inches;
        }
        updateStep("Turning 90 degrees");

        encoderDriveBasic(1.0, -turn90Inches, turn90Inches, 10.0);
        updateStep("Finished turning 90 degrees");
        motorExLeft.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidCurrent);
        motorExRight.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidCurrent);

    }


    public void turn90slow(Autonomous9533.TurnDirection direction) {
        //double turn90Inches = (3.543 * 3.1415) * (318/robot.REV_COUNTS_PER_MOTOR_REV);

        int ticks = 318;
        if(direction == Autonomous9533.TurnDirection.CLOCKWISE) {
            //maneuver build for counter-clockwise, so reverse
            ticks = -ticks;
        }
        updateStep("Turning 90 degrees");

        slowEncoderDrive(-ticks, ticks);
        updateStep("Finished turning 90 degrees");
    }

    public void slowEncoderDrive(int newLeftPosition, int newRightPosition) {
        double speed = 0.5;
        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        int newLeftTarget = robot.motorLeft.getCurrentPosition() + newLeftPosition;
        int newRightTarget = robot.motorRight.getCurrentPosition() + newRightPosition;
        robot.motorLeft.setTargetPosition(newLeftTarget);
        robot.motorRight.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while(opModeIsActive() && robot.isBusy()) {
            robot.setPower(speed , speed);
        }
        robot.stop();
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }



    public void encoderDriveBasic(double speed,
                                  double leftInches, double rightInches,
                                  double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        ElapsedTime runtime = new ElapsedTime();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.motorLeft.getCurrentPosition() + (int)(leftInches * robot.REV_COUNTS_PER_INCH);
            newRightTarget = robot.motorRight.getCurrentPosition() + (int)(rightInches * robot.REV_COUNTS_PER_INCH);
            robot.motorLeft.setTargetPosition(newLeftTarget);
            robot.motorRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
//            robot.motorLeft.setPower(Math.abs(speed));
//            robot.motorRight.setPower(Math.abs(speed));
            robot.setPower(Math.abs(speed), Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorLeft.isBusy() || robot.motorRight.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                        robot.motorLeft.getCurrentPosition(),
//                        robot.motorRight.getCurrentPosition());
//                telemetry.update();
            }

            // Stop all motion;
            robot.motorLeft.setPower(0);
            robot.motorRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        encoderDrive(speed, leftInches, rightInches, timeoutS, false);
    }

    /*
    *  Method to perform a relative move, based on encoder counts.
    *  Encoders are not reset as the move is based on the current position.
    *  Move will stop if any of three conditions occur:
    *  1) Move gets to the desired position
    *  2) Move runs out of time
    *  3) Driver stops the opmode running.
    */

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


        double maxSpeed = 0.95;
        if(speed > maxSpeed) {
            speed = maxSpeed;
        }
        int targetLeft, targetRight, currentRight, currentLeft;
        int differenceLeft, differenceRight;


        boolean accelerate = (Math.abs(leftInches) > 5) || (Math.abs(rightInches) > 5);
        boolean maxed = false;
        ElapsedTime runtime = new ElapsedTime();

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.setNewPosition(leftInches, rightInches);

            targetLeft = robot.motorLeft.getTargetPosition();
            targetRight =robot.motorRight.getTargetPosition();

            currentLeft = robot.motorLeft.getCurrentPosition();
            //int newLeftTarget = currentLeft + (int)(leftInches * robot.COUNTS_PER_INCH);

            int distanceToMoveTicks = targetLeft - currentLeft;


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

                currentLeft = robot.motorLeft.getCurrentPosition();
                currentRight = robot.motorRight.getCurrentPosition();
                differenceLeft = Math.abs(Math.abs(targetLeft) - Math.abs(currentLeft));

                if(accelerate) {
                    if (maxed) {

                        if(differenceLeft < distanceToMoveTicks * .5) {
                            currentSpeed = speed * .5;
                        }


//                        double newSpeed = 1 - (Easing.Interpolate(1 - (differenceLeft / scale), Easing.Functions.QuinticEaseIn));
//
//                        currentSpeed = speed * newSpeed;
//                        if (currentSpeed < 0.4) {
//                            currentSpeed = 0.4;
//                        }
                    } else if (currentSpeed < speed) {
                        multiplier = Easing.Interpolate(runtime.seconds() * 4, Easing.Functions.CubicEaseOut);
                        currentSpeed = speed * multiplier;
                    }

                } else {
                    currentSpeed = speed;
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


//    public void encoderDrive(double speed,
//                             double leftInches, double rightInches,
//                             double timeoutS, boolean holdPosition) {
//
//
//        int targetLeft, targetRight, currentRight, currentLeft;
//        int differenceLeft, differenceRight;
//
//        boolean maxed = false;
//        ElapsedTime runtime = new ElapsedTime();
//
//        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            robot.setNewPosition(leftInches, rightInches);
//
//            currentLeft = robot.motorLeft.getCurrentPosition();
//            int newLeftTarget = currentLeft + (int)(leftInches * robot.COUNTS_PER_INCH);
//
//            int scale = newLeftTarget - currentLeft;
//
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//
//            double currentSpeed = 0;
//            double multiplier = 0;
//
//
//            robot.setPower(currentSpeed , currentSpeed);
//
//
//            int lastPosition = 0;
//            double lastRuntime = 0;
//
//            int slowdownTick = 150;
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                    (runtime.seconds() < timeoutS) &&
//                    (robot.isBusy())) {
//                targetLeft = robot.motorLeft.getTargetPosition();
//                targetRight =robot.motorRight.getTargetPosition();
//                currentLeft = robot.motorLeft.getCurrentPosition();
//                currentRight = robot.motorRight.getCurrentPosition();
//                differenceLeft = Math.abs(Math.abs(targetLeft) - Math.abs(currentLeft));
//
//                if(maxed) {
//                    double newSpeed = Easing.Interpolate(1 - (differenceLeft / scale), Easing.Functions.QuinticEaseIn);
//                    currentSpeed = newSpeed;
//                    if(currentSpeed < 0.2) {
//                        currentSpeed = 0.2;
//                    }
//                }
//                else if(currentSpeed < speed) {
//                    multiplier = Easing.Interpolate(runtime.seconds() * 4, Easing.Functions.CubicEaseOut);
//                    currentSpeed = speed * multiplier;
//                }
//
//
//                telemetry.addLine()
//                        .addData("Multiplier", "%7f", multiplier)
//                        .addData("Speed", "%7f", currentSpeed);
//
//                //telemetry.update();
//
//                if(currentSpeed >= speed) {
//                    currentSpeed = speed;
//                    maxed = true;
//                }
//                robot.setPower(currentSpeed, currentSpeed);
//
//
//                // Display it for the driver.
//                telemetry.addLine().addData("Target",  "Running to %7d :%7d",
//                        targetLeft,
//                        targetRight);
//                telemetry.addLine().addData("Current",  "Running at %7d :%7d",
//                        currentLeft,
//                        currentRight);
//                telemetry.update();
//            }
//
//            if(holdPosition==false) {
//                // Stop all motion;
//                robot.stop();
//
//                // Turn off RUN_TO_POSITION
//                robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//        }
//    }


    protected void updateStep(String step) {
        currentStep = step;
        telemetry.update();
    }
}
