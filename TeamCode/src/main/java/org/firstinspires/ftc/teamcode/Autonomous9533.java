package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;

import java.util.Locale;

/**
 * Created by dmill on 10/28/2017.
 */

@Autonomous(name = "Autonomous", group = "Competition")
public class Autonomous9533 extends LinearOpMode {


    Robot robot = null;
    VuforiaHelper vuforiaHelper = null;
    Config config = null;


    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;

    private static final double distanceToCryptoBoxInchesFront = 33.0;
    private static final double distanceToCryptoBoxInchesBack = 0.0;
    private static final double cryptoBoxWidth = 7.5;


    String currentStep = "";

    double distanceToDrive = 0;
    int leftPosition;
    int rightPosition;


    @Override
    public void runOpMode() throws InterruptedException {

        config = new Config(hardwareMap.appContext);
        config.Read();

        telemetry.addData("Color:", config.color.toString());
        telemetry.addData("Position:", config.position.toString());
        telemetry.addData("Park:", config.Park);
        telemetry.addData("Jewel:", config.JewelKnockOff);
        telemetry.addData("Crypto:", config.CryptoBox);
        telemetry.update();


        robot = new Robot(hardwareMap);
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.stop();

        vuforiaHelper = new VuforiaHelper();
        vuforiaHelper.initVuforia(hardwareMap);

        composeTelemetry();
        waitForStart();





        runProgram();


        while(opModeIsActive()) {
            idle();
            telemetry.update();
        }


    }

    void updateStep(String step) {
        currentStep = step;
        telemetry.update();
    }

    void runProgram() {

        if(config.JewelKnockOff) {
            updateStep("Knock off jewel");
            knockOffJewel();
        }

        if(config.position == Config.Positions.FRONT) {
            runProgramFront();
        } else {
            runProgramBack();
        }

//        if(config.CryptoBox) {
//
//            distanceToDrive = (config.position == Config.Positions.FRONT) ?
//                    distanceToCryptoBoxInchesFront :
//                    distanceToCryptoBoxInchesBack;
//
//            vuMark = detectVuMark(5.0);
//            if(vuMark == RelicRecoveryVuMark.CENTER) {
//                distanceToDrive += cryptoBoxWidth;
//            }
//
//            if(vuMark != RelicRecoveryVuMark.LEFT) {
//                distanceToDrive += cryptoBoxWidth*2;
//            }
//        }
//
//        if(config.JewelKnockOff) {
//            // do jewel knock off
//
//        }
//
//        if(config.Park) {
//            // park robot
//            // drive forward 45 inches, taking no more than 10 seconds
//            double inches = distanceToCryptoBoxInchesFront;
//            double timeoutS = 10.0;
//            encoderDrive(0.7, inches, inches, timeoutS);
//        }
    }

    void runProgramFront() {

        // read pictograph
        if(config.CryptoBox) {
            vuMark = detectVuMark(5.0);
        }

    }
    void runProgramBack() {

    }



    RelicRecoveryVuMark detectVuMark(double timeout) {
        ElapsedTime runtime = new ElapsedTime();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        while (opModeIsActive() && runtime.seconds() < timeout) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            vuMark = vuforiaHelper.detectVuMark();


            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                break;
            }
            else {
                telemetry.addData("VuMark", "not visible");

            }

            telemetry.update();
        }//while loop
        return vuMark;
    }


    /*
    *  Method to perfmorm a relative move, based on encoder counts.
    *  Encoders are not reset as the move is based on the current position.
    *  Move will stop if any of three conditions occur:
    *  1) Move gets to the desired position
    *  2) Move runs out of time
    *  3) Driver stops the opmode running.
    */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        ElapsedTime runtime = new ElapsedTime();

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.setNewPosition(leftInches, rightInches);


            // reset the timeout time and start motion.
            runtime.reset();

            double currentSpeed = 0;
            double multiplier = 0;

            robot.setPower(currentSpeed , currentSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.isBusy())) {


                if(currentSpeed < speed) {
                    multiplier = Easing.Interpolate(runtime.seconds(), Easing.Functions.CubicEaseOut);
                    currentSpeed = speed * multiplier;
                }

                telemetry.addLine()
                        .addData("Multiplier", "%7f", multiplier)
                        .addData("Speed", "%7f", currentSpeed);

                //telemetry.update();

                if(currentSpeed > speed) {
                    currentSpeed = speed;
                }
                robot.setPower(currentSpeed, currentSpeed);


                // Display it for the driver.
                telemetry.addLine().addData("Path1",  "Running to %7d :%7d",
                        robot.motorLeft.getTargetPosition(),
                        robot.motorRight.getTargetPosition());
                telemetry.addLine().addData("Path2",  "Running at %7d :%7d",
                        robot.motorLeft.getCurrentPosition(),
                        robot.motorRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.stop();

            // Turn off RUN_TO_POSITION
            robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }




    void knockOffJewel() {

        double movement = 2.0; //how far to move in inches
        double speed = 0.4; //how fast to move
        boolean moveForward = false;
        double timeoutS = 3.0; //how long before timing out movement



        updateStep("Extending Arm");
        robot.extendColorArm();
        waitForTick(1000);


        updateStep("Sensing color");
        Robot.ColorSensed color = robot.SenseJewel();

        // always move towards the opposite of team color
        if(color == Robot.ColorSensed.RED) {
            moveForward = config.color == Config.Colors.BLUE;

        } else if (color == Robot.ColorSensed.BLUE) {
            moveForward = config.color == Config.Colors.RED;

        } else {
            updateStep("Retracting Arm - No color");
            robot.retractColorArm();
            waitForTick(1000);
            return;
        }


        if(moveForward == false) { //negate movement if moving backwards
            movement *= -1;
        }
        updateStep("Moving to knock off jewel");
        encoderDrive(speed, movement, movement, timeoutS);
        waitForTick(1000);
        updateStep("Retracting Arm");
        robot.retractColorArm();
        waitForTick(1000);
        updateStep("Moving back to start");
        encoderDrive(speed, -movement, -movement, timeoutS);

    }



    public void waitForTick(long periodMs) {

        // sleep for the remaining portion of the regular cycle period.
        if ( opModeIsActive()) {
            try {
                Thread.sleep(periodMs);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        //runtime.reset();
    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
//            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            gravity  = imu.getGravity();
            leftPosition = robot.motorLeft.getCurrentPosition();
            rightPosition = robot.motorRight.getCurrentPosition();

        }
        });

        telemetry.addLine()
                .addData("Step",new Func<String>() {
                    @Override public String value() {
                        return currentStep;
                    }
                });


        if(config.CryptoBox) {
            telemetry.addLine()
                    .addData("VuMark",new Func<String>() {
                        @Override public String value() {
                            return vuMark.toString();
                        }
                    });
        }

        telemetry.addLine()
                .addData("Left",new Func<String>() {
                    @Override public String value() {
                        return Integer.toString(leftPosition);
                    }
                });

        telemetry.addLine()
                .addData("Right",new Func<String>() {
                    @Override public String value() {
                        return Integer.toString(rightPosition);
                    }
                });



    }
}
