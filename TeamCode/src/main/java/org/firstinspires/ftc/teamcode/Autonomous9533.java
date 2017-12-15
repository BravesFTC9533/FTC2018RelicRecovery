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

    private static final double cryptoBoxWidth = 7.5;
    private static final long pauseTimeBetweenSteps = 250;
    private double speed = 1.0;
    private double fast_speed = 1.0;

    String currentStep = "";

    double distanceToDrive = 0;
    int leftPosition;
    int rightPosition;

    enum TurnDirection {
        CLOCKWISE,
        COUNTERCLOCKWISE
    }

    @Override
    public void runOpMode() throws InterruptedException {


        robot = new Robot(hardwareMap);
        robot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.stop();

        config = new Config(hardwareMap.appContext);
        config.Read();

        speed = config.speed;


        telemetry.addData("**** PLEASE WAIT FOR VUFORIA TO INIT ****", "");

        telemetry.addData("Color:", config.color.toString());
        telemetry.addData("Position:", config.position.toString());
        telemetry.addData("Park:", config.Park);
        telemetry.addData("Jewel:", config.JewelKnockOff);
        telemetry.addData("Crypto:", config.CryptoBox);

        telemetry.update();






        vuforiaHelper = new VuforiaHelper();
        vuforiaHelper.initVuforia(hardwareMap);

        telemetry.addData("Vuforia:", "Initialized!");
        telemetry.update();

        composeTelemetry();
        waitForStart();


        if(config.delayStart > 0.0) {
            ElapsedTime runtime = new ElapsedTime();
            while(opModeIsActive() && runtime.seconds() < config.delayStart) {

            }
        }

        runProgram();

        if(opModeIsActive()) {
            robot.GrabberStart();
            robot.retractColorArm();
        }


    }

    void updateStep(String step) {
        currentStep = step;
        telemetry.update();
    }

    void readPictograph() {

        // read pictograph

        updateStep("Detecting VuMark");
        vuMark = detectVuMark(2.0);

        if (vuMark == RelicRecoveryVuMark.CENTER) {

            distanceToDrive += cryptoBoxWidth;

        } else {

            if (config.color == Config.Colors.RED) {

                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    distanceToDrive += cryptoBoxWidth * 2;
                }

            } else {

                if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    distanceToDrive += cryptoBoxWidth * 2;
                }
            }
        }
        updateStep("Finished detecting VuMark");

    }

    void knockOffJewel() {

        double movement = 2.5; //how far to move in inches
        double speed = 0.35; //how fast to move
        boolean moveForward = false;
        double timeoutS = 4.0; //how long before timing out movement



        updateStep("Extending Arm");
        robot.extendColorArm();
        waitForTick(1000);


        updateStep("Sensing color");
        Robot.ColorSensed color = robot.SenseJewel();

        // always move towards the opposite of team color
        if(color == Robot.ColorSensed.RED) {
            moveForward = config.color == Config.Colors.RED;

        } else if (color == Robot.ColorSensed.BLUE) {
            moveForward = config.color == Config.Colors.BLUE;

        } else {
            updateStep("Retracting Arm - No color");
            robot.retractColorArm();
            waitForTick(1000);
            return;
        }


        if(moveForward == false) { //negate movement if moving backwards
            movement *= -1;
        }

        double left = robot.motorLeft.getCurrentPosition();
        double right = robot.motorRight.getCurrentPosition();

        updateStep("Moving to knock off jewel");
        encoderDrive(speed, movement, movement, timeoutS, true);

        waitForTick(750);
        updateStep("Retracting Arm");
        robot.retractColorArm();
        waitForTick(750);
        updateStep("Moving back to start");

//        double moveLeft = (left - robot.motorLeft.getCurrentPosition()) / robot.COUNTS_PER_INCH;
//        double moveRight = (right - robot.motorRight.getCurrentPosition()) / robot.COUNTS_PER_INCH;

        encoderDrive(speed, -movement, -movement, timeoutS, true);
        //encoderDrive(speed, moveLeft, moveRight, timeoutS, true);

    }

    void backUp(double distance) {

        pause();
        updateStep("Back up");
        encoderDrive(0.5, -distance, -distance, 5.0);
        updateStep("Finished Back up");
    }

    void runProgram() {

        if(config.CryptoBox && opModeIsActive()) {
            updateStep("Reading pictograph");
            readPictograph();

            // grab block and lift
            updateStep("Grabbing block");
            robot.GrabberGrab();
            waitForTick(500);

            updateStep("Lifting block");
            encoderLift(7);


            //updateStep("Moving into position to knock off jewel");
            //move into position to sense jewel color
//            double distanceForSensor = 2.0;
//            encoderDrive(0.4, distanceForSensor, distanceForSensor, 4, true);

        }

        pause();

        if(config.JewelKnockOff && opModeIsActive()) {
            updateStep("Knock off jewel");
            knockOffJewel();
            updateStep("Finished knock off jewel");
        }

        pause();

        if(config.position == Config.Positions.FRONT) {
            runProgramFront();
        } else {
            runProgramBack();
        }

    }

    void pause() {
        waitForTick(pauseTimeBetweenSteps);
    }

    void turn90(TurnDirection direction) {
        double turn90Inches = (3.543 * 3.1415) * 1.09;

        if(direction == TurnDirection.CLOCKWISE) {
            //maneuver build for counter-clockwise, so reverse
            turn90Inches = -turn90Inches;
        }
        updateStep("Turning 90 degrees");

        encoderDrive(speed, -turn90Inches, turn90Inches, 3.0);
        updateStep("Finished turning 90 degrees");
    }

    void parkFromFront() {
        distanceToDrive += (config.color == Config.Colors.RED) ? config.distanceToCryptoBoxInchesFrontRed : config.distanceToCryptoBoxInchesFrontBlue;

        updateStep("Doing Park maneuver");

        if(config.color == Config.Colors.RED) {
            //red has to drive backwards, so invert distance
            distanceToDrive = -distanceToDrive;
        }

        encoderDrive(speed, distanceToDrive, distanceToDrive, 7.0);
        updateStep("Finished park maneuver");

        pause();

        turn90(TurnDirection.COUNTERCLOCKWISE);

        pause();
        double placeBlockDistance = 15;
        updateStep("Parking");
        encoderDrive(speed, placeBlockDistance, placeBlockDistance, 5.0);
        updateStep("Finished Parking");
    }

    void parkFromBack() {

        double distance = (config.color == Config.Colors.BLUE ) ? config.distanceToDriveOffBalanceBoardBackBlue : config.distanceToDriveOffBalanceBoardBackRed;

        if(config.color == Config.Colors.RED) {
            //red has to drive backwards, so invert distance
            distance = -distance;
        }
        updateStep("Driving off of balance board");
        encoderDrive(speed, distance, distance, 7.0);
        updateStep("Finished driving off of balance board");
        pause();

        turn90(TurnDirection.CLOCKWISE);
        pause();


        //move backwards to wall
        double distanceToWall = 15.0; //this should be pretty static if not overkill for red
        encoderDrive(speed, -distanceToWall, -distanceToWall, 5.0);
        pause();

        updateStep("Lower block some");
        encoderLift(1.0);
        pause();

        //move to correct position in front of crypto wall
        double distanceToFirstBox = (config.color == Config.Colors.RED) ? config.distanceToCryptoBoxInchesBackRed : config.distanceToCryptoBoxInchesBackBlue;
        distance = distanceToFirstBox + distanceToDrive;
        encoderDrive(fast_speed, distance, distance, 5.0);
        pause();

        TurnDirection dir = (config.color == Config.Colors.RED) ? TurnDirection.CLOCKWISE : TurnDirection.COUNTERCLOCKWISE;
        turn90(dir);
        pause();


        double distanceIntoWall = 10.0;
        encoderDrive(speed, distanceIntoWall, distanceIntoWall, 5.0);
        pause();


    }

    void dropCryptoblock() {


        backUp(1.0);
        pause();

        encoderLift(1.0);
        updateStep("Drop block");
        robot.GrabberLoose();
        updateStep("Finished drop block");

        encoderLift(0.0);
//        updateStep("Lower block");
//        robot.GrabberLiftLower();
//        waitForTick(300);
//        robot.GrabberLiftStop();
        updateStep("Finished lower block");

        pause();

        robot.GrabberStart();
        backUp(2.0);
        //backup negative is move forward..
        backUp(-5.0);
        backUp(2.0);

        //        while(opModeIsActive()) {
//            robot.GrabberOpen();
//            backUp(2.0);
//            robot.GrabberStart();
//            backUp(-2.0);
//        }

    }

    void runProgramFront() {

        if(config.Park && opModeIsActive()){
            parkFromFront();
        }

        if(config.CryptoBox && opModeIsActive()) {
            dropCryptoblock();
        }

    }

    void runProgramBack() {

        if(config.Park) {
            parkFromBack();
        }

        if(config.CryptoBox) {
            dropCryptoblock();
        }
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


    public void encoderLift(double inches) {
        ElapsedTime lifttime = new ElapsedTime();

        if(opModeIsActive()) {
            robot.setLiftPosition(inches);

            lifttime.reset();
            robot.motorLift.setPower(robot.LIFTSPEED);
            while(opModeIsActive() && lifttime.seconds() < 2 && robot.motorLift.isBusy()) {

            }
            if(inches == 0) {
                robot.GrabberLiftStop();
            }
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
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, boolean holdPosition) {
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
                    multiplier = Easing.Interpolate(runtime.seconds() / 2, Easing.Functions.CubicEaseOut);
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
                telemetry.addLine().addData("Target",  "Running to %7d :%7d",
                        robot.motorLeft.getTargetPosition(),
                        robot.motorRight.getTargetPosition());
                telemetry.addLine().addData("Current",  "Running at %7d :%7d",
                        robot.motorLeft.getCurrentPosition(),
                        robot.motorRight.getCurrentPosition());
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
