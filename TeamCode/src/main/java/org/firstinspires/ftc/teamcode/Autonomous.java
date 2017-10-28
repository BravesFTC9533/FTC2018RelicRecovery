package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by dmill on 10/28/2017.
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous", group = "Competition")
public class Autonomous extends LinearOpMode {


    Robot robot = null;



    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        waitForStart();



        //addTelemetry("About to drive forward");
        double inches = 45.0;
        double timeoutS = 10.0;
        encoderDrive(0.5, inches, inches, timeoutS);

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

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.setNewPosition(leftInches, rightInches);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.setPower(Math.abs(speed), Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.isBusy())) {

                // Display it for the driver.
//                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
//                telemetry.addData("Path2",  "Running at %7d :%7d",
//                        motorLeft.getCurrentPosition(),
//                        motorRight.getCurrentPosition());
//                telemetry.update();
            }

            // Stop all motion;
            robot.stop();

            // Turn off RUN_TO_POSITION
            robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //  sleep(250);   // optional pause after each move
        }
    }
}
