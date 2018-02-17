package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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


    public void turn90(Autonomous9533.TurnDirection direction) {
        double turn90Inches = (3.51 * 3.1415) * (313/robot.REV_COUNTS_PER_MOTOR_REV);

        if(direction == Autonomous9533.TurnDirection.CLOCKWISE) {
            //maneuver build for counter-clockwise, so reverse
            turn90Inches = -turn90Inches;
        }
        updateStep("Turning 90 degrees");
        encoderDrive(0.5, -turn90Inches, turn90Inches, 10.0);
        updateStep("Finished turning 90 degrees");

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


    public void encoderDrive_straight(double targetSpeed, double inches, double timeoutS) {
        boolean atMaxSpeed = false;
        ElapsedTime runtime = new ElapsedTime();

        final double Kp = 0.1;
//        final double Ki = 0.01;
//        final double Kd = 0.01;

        double currentSpeed = 0;
        double minSpeed = 0.25;

        int lastError = 0;
        int error = 0;

        if(opModeIsActive()) {

            Pair<Integer, Integer> target = robot.setNewPosition(inches); // set robots new target, and set mode to run to position
            Pair<Integer, Integer> start = robot.getCurrentPosition();    // get robots starting position

            boolean shouldLoopContinue = opModeIsActive();

            while(shouldLoopContinue){

                Pair<Integer, Integer> current = robot.getCurrentPosition(); //get robots current position

                if(!atMaxSpeed) {
                    //accelerate until we reach our target speed
                    currentSpeed = accelerate(0.25, currentSpeed, targetSpeed);
                    if(currentSpeed >= targetSpeed) {
                        currentSpeed = targetSpeed;
                        atMaxSpeed = true;
                    }
                } else {
                    //use deceleration curve CubicEaseIn
                    //only using left wheel position
                    currentSpeed = decelerate(start.getLeft(), target.getLeft(), current.getLeft(), currentSpeed);
                    if(currentSpeed <= minSpeed){
                        currentSpeed = minSpeed;
                    }
                }

                //calculate error (has one wheel moved further than another)
                int rightPos = (target.getRight() - current.getRight());
                int leftPos = (target.getLeft() - current.getLeft());
                error =  rightPos - leftPos;

                //set left and right powers based on error (P loop)
                //TODO: tweak Kp component
                double leftSpeed = Range.clip(currentSpeed + (Kp*error), targetSpeed*-1, targetSpeed);
                double rightSpeed = Range.clip(currentSpeed - (Kp*error), targetSpeed-1, targetSpeed);


                robot.setPower(leftSpeed, rightSpeed);


                shouldLoopContinue =
                        runtime.seconds() < timeoutS &&
                        robot.isBusy() &&
                        opModeIsActive();
            }

            robot.stop();
            robot.setRunUsingEncoders();
        }

    }

    public double accelerate(double accelerationSeconds, double currentSeconds, double targetSpeed){
        double percent = currentSeconds * (1/accelerationSeconds);
        return accelerate(percent, targetSpeed);
    }

    public double accelerate(double percent, double targetSpeed) {
        double multiplier = Easing.Interpolate(percent, Easing.Functions.CubicEaseOut);
        return (targetSpeed * multiplier);
    }

    public double decelerate(int startPosition, int targetPosition, int currentPosition, double currentSpeed) {

        double totalDistance = (double)(Math.abs(targetPosition) - Math.abs(startPosition));
        double distanceLeftToTravel = (double)(Math.abs(targetPosition) - Math.abs(currentPosition));

        double percent = 1 - (distanceLeftToTravel / totalDistance);
        return decelerate(percent, currentSpeed);

    }

    public double decelerate(double percent, double currentSpeed) {
        double multiplier = (Easing.Interpolate(percent, Easing.Functions.CubicEaseIn));
        return (currentSpeed * multiplier);
    }

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

            int scale = Math.abs(Math.abs(targetLeft) - Math.abs(currentLeft));

            //int newLeftTarget = currentLeft + (int)(leftInches * robot.COUNTS_PER_INCH);

            int distanceToMoveTicks = targetLeft - currentLeft;


            // reset the timeout time and start motion.
            runtime.reset();

            double currentSpeed = 0;
            double multiplier = 0;


            robot.setPower(currentSpeed , currentSpeed);


            int lastPosition = 0;
            double lastRuntime = 0;
            double easein = 0;

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

                        easein = (Easing.Interpolate(1 - ((double)differenceLeft / (double)scale), Easing.Functions.CubicEaseIn));
                        multiplier = 1 - easein;

                        currentSpeed = speed * multiplier;
                        if (currentSpeed < 0.4) {
                            currentSpeed = 0.4;
                        }
                    } else if (currentSpeed < speed) {
                        multiplier = Easing.Interpolate(runtime.seconds() * 4, Easing.Functions.CubicEaseOut);
                        currentSpeed = speed * multiplier;
                    }

                } else {
                    currentSpeed = speed;
                }

                telemetry.addLine()
                        .addData("Scale", "%4d", scale)
                        .addData("CL", "%4d", Math.abs(currentLeft))
                        .addData("DL", "%4d", differenceLeft);

                telemetry.addLine()
                        .addData("Multiplier", "%7f", multiplier)
                        .addData("Speed", "%7f", currentSpeed)
                        .addData("Ease", "%7f", easein);

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





    protected void updateStep(String step) {
        currentStep = step;
        telemetry.update();
    }
}
