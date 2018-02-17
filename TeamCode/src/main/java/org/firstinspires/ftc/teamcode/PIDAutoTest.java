package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by dmill on 2/3/2018.
 */
@Autonomous(name = "PID Auto Test", group = "Testing")
@Disabled
public class PIDAutoTest extends LinearOpMode9533 {




    DcMotorEx motorExLeft;
    DcMotorEx motorExRight;



    Config config = null;

    private static final double cryptoBoxWidth = 7.5;
    private static final long pauseTimeBetweenSteps = 100;


    public static double NEW_P = 1;
    public static double NEW_I = 0.0;
    public static double NEW_D = 0.0;
    public static SimpleMenu menu = new SimpleMenu("Autonomous Menu");


    @Override
    public void runOpMode() throws InterruptedException {


        robot = new Robot(hardwareMap);
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.stop();

        motorExLeft = (DcMotorEx)robot.motorLeft;
        motorExRight = (DcMotorEx)robot.motorRight;

        // get the PID coefficients for the RUN_USING_ENCODER  modes.
        PIDCoefficients pidOrigLeft = motorExLeft.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDCoefficients pidOrigRight = motorExRight.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        // re-read coefficients and verify change.

        motorExLeft.setTargetPositionTolerance(2);
        motorExRight.setTargetPositionTolerance(2);

        robot.motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        menu.clearOptions();
        menu.addOption("P", 40, 0, 0.01, NEW_P);
        menu.addOption("I", 40, 0, 0.01, NEW_I);
        menu.addOption("D", 40, 0, 0.01, NEW_D);
        menu.setGamepad(gamepad1);
        menu.setTelemetry(telemetry);


        waitForStart();


        while(opModeIsActive()) {

            if (gamepad1.start) {
                break;
            }

            menu.displayMenu();

            NEW_P = Double.parseDouble(menu.getCurrentChoiceOf("P"));
            NEW_I = Double.parseDouble(menu.getCurrentChoiceOf("I"));
            NEW_D = Double.parseDouble(menu.getCurrentChoiceOf("D"));

        }


        // change coefficients using methods included with DcMotorEx class.
        PIDCoefficients pidNew = new PIDCoefficients(NEW_P, NEW_I, NEW_D);
        motorExLeft.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);
        motorExRight.setPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);


        PIDCoefficients pidModRight = motorExRight.getPIDCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        DriveLoop();





        while(opModeIsActive()) {

            telemetry.addData("Runtime", "%.03f", getRuntime());

            telemetry.addData("Position Left", "%d", robot.motorLeft.getCurrentPosition());
            telemetry.addData("Position Right", "%d", robot.motorRight.getCurrentPosition());


            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f",
                    pidOrigLeft.p, pidOrigLeft.i, pidOrigLeft.d);
            telemetry.addData("P,I,D (modified)", "%.04f, %.04f, %.04f",
                    pidModRight.p, pidModRight.i, pidModRight.d);
            telemetry.update();
        }


    }

    public void DriveLoop() {
        encoderDriveBasic(0.5, 45, 45, 10.0);


        for(int i = 0;i<2;i++) {

            robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(100);
            turn90(Autonomous9533.TurnDirection.CLOCKWISE);
            sleep(1000);

        }

        encoderDriveBasic(0.5, 45, 45, 10.0);
    }

    public void HoldPosition (){

        double inches = 1.0;

        // Determine new target position, and pass to motor controller
        int newLeftTarget = robot.motorLeft.getCurrentPosition() + (int)(inches * robot.REV_COUNTS_PER_INCH);
        int newRightTarget = robot.motorRight.getCurrentPosition() + (int)(inches * robot.REV_COUNTS_PER_INCH);
        robot.motorLeft.setTargetPosition(newLeftTarget);
        robot.motorRight.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        robot.motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.setPower(1.0, 1.0);

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
            robot.setRunToPosition();

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
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.motorLeft.getCurrentPosition(),
                        robot.motorRight.getCurrentPosition());
                telemetry.update();
            }

            robot.stop();
            robot.setRunUsingEncoders();
//            // Stop all motion;
//            robot.motorLeft.setPower(0);
//            robot.motorRight.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
        }
    }



}
