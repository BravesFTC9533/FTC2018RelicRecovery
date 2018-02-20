package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous9533;
import org.firstinspires.ftc.teamcode.Easing;
import org.firstinspires.ftc.teamcode.FtcGamePad;
import org.firstinspires.ftc.teamcode.IDrive;
import org.firstinspires.ftc.teamcode.Pair;
import org.firstinspires.ftc.teamcode.Quad;


/**
 * Created by dmill on 2/17/2018.
 */

public abstract class LinearOpModeV2_9533 extends LinearOpMode implements FtcGamePad.ButtonHandler {

    protected FtcGamePad driverGamepad;
    protected FtcGamePad operatorGamepad;
    protected RobotV2 robot;

    protected IDrive robotDrive;

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable


    enum TurnDirection {
        CLOCKWISE,
        COUNTERCLOCKWISE
    }

    protected void initialize(){
        robot = new RobotV2(hardwareMap);
        driverGamepad = new FtcGamePad("DriverGamepad", gamepad1, this);
        operatorGamepad = new FtcGamePad("OperatorGamepad", gamepad2, this);

        this.robotDrive = getRobotDrive(robot, driverGamepad);

        robot.updatePID(10, 10, 1);
        robot.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    protected void updateGamepads() {
        driverGamepad.update();
        operatorGamepad.update();
    }




    public void gamepadButtonEvent(FtcGamePad gamepad, int button, boolean pressed) {
        if(gamepad == driverGamepad) {
            handleDriverGamepad(gamepad, button, pressed);
        } else if(gamepad == operatorGamepad) {
            handleOperatorGamepad(gamepad, button, pressed);
        }
    }



    protected abstract IDrive getRobotDrive(RobotV2 robot, FtcGamePad gamePad);
    protected abstract void handleDriverGamepad(FtcGamePad gamepad, int button, boolean pressed);
    protected abstract void handleOperatorGamepad(FtcGamePad gamepad, int button, boolean pressed);




    public void encoderDrive_straight(double targetSpeed, double inches, double timeoutS) {
        boolean atMaxSpeed = false;
        ElapsedTime runtime = new ElapsedTime();

        double currentSpeed = 0;
        double minSpeed = 0.25;

        int lastError = 0;
        int error = 0;

        if(opModeIsActive()) {

            Quad<Integer, Integer, Integer, Integer> target = robot.setNewPosition(inches); // set robots new target, and set mode to run to position
            Quad<Integer, Integer, Integer, Integer> start = robot.getCurrentPosition();    // get robots starting position

            boolean shouldLoopContinue = opModeIsActive();

            while(shouldLoopContinue){

                Quad<Integer, Integer, Integer, Integer> current = robot.getCurrentPosition(); //get robots current position

                if(!atMaxSpeed) {
                    //accelerate until we reach our target speed
                    currentSpeed = accelerate(0.25, runtime.seconds(), targetSpeed);
                    if(currentSpeed >= targetSpeed) {
                        currentSpeed = targetSpeed;
                        atMaxSpeed = true;
                    }
                } else {
                    if(currentSpeed > minSpeed) {
                        //use deceleration curve CubicEaseIn
                        //only using left wheel position
                        currentSpeed = decelerate(start.getA(), target.getA(), current.getA(), currentSpeed);
                        if (currentSpeed <= minSpeed) {
                            currentSpeed = minSpeed;
                        }
                    }
                }


                double ly, lx, rx;


                robotDrive.drive(currentSpeed, 0, 0);


                telemetry.addData("Current Speed", "%2f", currentSpeed);
                telemetry.addLine("Pos")
                        .addData("T", "%d", target.getA())
                        .addData("C", "%d", current.getA());

                telemetry.update();


                shouldLoopContinue =
                        runtime.seconds() < timeoutS &&
                                robot.isBusy() &&
                                opModeIsActive();
            }

            robot.stop();
            robot.setRunUsingEncoders();
        }

    }

    /*
        this is only useful for forward, back, turn
     */
    public void encoderDriveTicks(double targetSpeed, int leftTicks, int rightTicks, double timeoutS) {
        boolean atMaxSpeed = false;
        ElapsedTime runtime = new ElapsedTime();

        double currentSpeed = 0;
        double minSpeed = 0.25;

        int error = 0;

        if(opModeIsActive()) {

            Quad<Integer, Integer, Integer, Integer> target = robot.setNewPositionTicks(leftTicks, rightTicks, leftTicks, rightTicks); // set robots new target, and set mode to run to position
            Quad<Integer, Integer, Integer, Integer> start = robot.getCurrentPosition();    // get robots starting position

            boolean shouldLoopContinue = opModeIsActive();

            while(shouldLoopContinue){

                Quad<Integer, Integer, Integer, Integer> current = robot.getCurrentPosition(); //get robots current position

                if(!atMaxSpeed) {
                    //accelerate until we reach our target speed
                    currentSpeed = accelerate(0.25, runtime.seconds(), targetSpeed);
                    if(currentSpeed >= targetSpeed) {
                        currentSpeed = targetSpeed;
                        atMaxSpeed = true;
                    }
                } else {
                    //use deceleration curve CubicEaseIn
                    //only using left wheel position
                    currentSpeed = decelerate(start.getA(), target.getA(), current.getA(), currentSpeed);
                    if(currentSpeed <= minSpeed){
                        currentSpeed = minSpeed;
                    }
                }


                double ly, lx, rx;


                robot.Drive(currentSpeed, currentSpeed, currentSpeed, currentSpeed);
                //robotDrive.drive(currentSpeed, 0, 0);


                telemetry.addData("Current Speed", "%2f", currentSpeed);
                telemetry.addLine("Pos")
                        .addData("T", "%d", target.getA())
                        .addData("C", "%d", current.getA());

                telemetry.update();


                shouldLoopContinue =
                        runtime.seconds() < timeoutS &&
                                robot.isBusy(true) &&
                                opModeIsActive();
            }

            robot.stop();
            robot.setRunUsingEncoders();
        }

    }


    public void turn90(TurnDirection direction, int ticks) {

        //double turn90Inches = (4 * Math.PI) * (counts/(RobotV2.REV_COUNTS_PER_MOTOR_REV * RobotV2.DRIVE_GEAR_REDUCTION);

        if(direction == TurnDirection.CLOCKWISE) {
            //maneuver build for counter-clockwise, so reverse
            ticks = -ticks;
        }
        //updateStep("Turning 90 degrees");
        //encoderDrive(0.5, -turn90Inches, turn90Inches, 10.0);

        encoderDriveTicks(0.5, -ticks, ticks, 2.0);
        //updateStep("Finished turning 90 degrees");
    }
    public void turn90(TurnDirection direction) {
        turn90(direction, 200);
    }




    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
    /**
     * Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
//            rightSpeed  = speed * steer;
//            leftSpeed   = -rightSpeed;
        }

        robotDrive.drive(0, 0, steer * speed);

        // Send desired speeds to motors.
//        robot.leftDrive.setPower(leftSpeed);
//        robot.rightDrive.setPower(rightSpeed);



        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f", steer);

        return onTarget;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;


        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - angles.firstAngle;
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }


    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
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
        double multiplier = 1 - (Easing.Interpolate(percent, Easing.Functions.QuinticEaseIn));
        return (currentSpeed * multiplier);
    }


}
