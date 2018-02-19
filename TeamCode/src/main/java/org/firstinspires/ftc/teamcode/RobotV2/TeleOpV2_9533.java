package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.FtcGamePad;
import org.firstinspires.ftc.teamcode.IDrive;
import org.firstinspires.ftc.teamcode.SimpleMenu;

import java.util.ArrayList;
import java.util.Locale;

/**
 * Created by dmill on 2/17/2018.
 */
@TeleOp(name = "TeleOp V2", group = "Competition")
public class TeleOpV2_9533 extends LinearOpModeV2_9533 {

    public static SimpleMenu menu = new SimpleMenu("Autonomous Menu");

    public static int ticks = 200;


    boolean flipperButtonPressed = false;

    boolean liftDownPressed = false;
    boolean liftUpPressed = false;

    double lastFlipperFloop = 0;


    ElapsedTime runtime = new ElapsedTime();

    ArrayList<Integer> positions;// = robot.getPositions();
    @Override
    public void runOpMode() throws InterruptedException {

        initialize();


//        menu.clearOptions();
//        menu.addOption("Ticks", 250, 150, 1, ticks);
//
//        menu.setGamepad(gamepad1);
//        menu.setTelemetry(telemetry);

        ComposeTelemetry();

        waitForStart();

        robot.updatePID(10, 10, 1);
        robot.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        runtime.reset();
        while(opModeIsActive()) {

//            menu.displayMenu();
//            ticks = (int)Double.parseDouble(menu.getCurrentChoiceOf("Ticks"));

            updateGamepads();
            robotDrive.handle();

            handleFlipper();

            handleLift();

            telemetry.update();


        }

    }

    private  void handleLift() {
        if(liftDownPressed){
            if(robot.touchSensorPressed()) {
                robot.stopLift();
            } else {
                robot.lowerLift();
            }
        } else if(liftUpPressed) {
            robot.raiseLift();
        } else {
            robot.stopLift();
        }
    }

    private void handleFlipper() {
        if(flipperButtonPressed) {

            boolean shouldIncrement = lastFlipperFloop == 0 || runtime.milliseconds() >= lastFlipperFloop + robot.servoDelayDelta;
            if(shouldIncrement) {
                robot.incrementBlockFlipperServo();
                lastFlipperFloop = runtime.milliseconds();
            }

        } else {
            boolean shouldDecrement = lastFlipperFloop == 0 || runtime.milliseconds() >= lastFlipperFloop + robot.servoDelayDelta;
            if(shouldDecrement) {
                robot.closeBlockFlipperServo();
                lastFlipperFloop = runtime.milliseconds();
            }

        }
    }


    Orientation angles;

    private void ComposeTelemetry() {

        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.

            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            positions = robot.getPositions();
        }
        });

        telemetry.addLine()
                .addData("Runtime", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("%f", runtime.milliseconds());
                    }
                })
                .addData("Last", new Func<String>() {
                    @Override
                    public String value() {
                        return String.format("%f", lastFlipperFloop);
                    }
                })
                .addData("Servo", new Func<String>() {
                    @Override
                    public String value() {
                        return formatDegrees(robot.getServoPosition());
                    }
                });

        telemetry.addLine("Positions")
                .addData("FL", new Func<String>() {
                    @Override public String value() {
                        return positions.get(0).toString();
                    }
                })
                .addData("FR", new Func<String>() {
                    @Override public String value() {
                        return positions.get(1).toString();
                    }
                })
                .addData("RL", new Func<String>() {
                    @Override public String value() {
                        return positions.get(2).toString();
                    }
                })
                .addData("RR", new Func<String>() {
                    @Override public String value() {
                        return positions.get(3).toString();
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
    }



    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    String formatDouble(double value) {
        return  String.format(Locale.getDefault(), "%.2f", value);
    }

    @Override
    protected IDrive getRobotDrive(RobotV2 robot, FtcGamePad gamePad) {
        return new ComplicatedMecanumDrive_B(robot, gamePad);
    }

    @Override
    protected void handleDriverGamepad(FtcGamePad gamepad, int button, boolean pressed) {

        switch (button) {
            case FtcGamePad.GAMEPAD_A:

                break;
            case FtcGamePad.GAMEPAD_B:


                break;
            case FtcGamePad.GAMEPAD_X:

                break;
            case FtcGamePad.GAMEPAD_Y:
                break;
            case FtcGamePad.GAMEPAD_DPAD_DOWN:
                break;
            case FtcGamePad.GAMEPAD_DPAD_LEFT:
                break;
            case FtcGamePad.GAMEPAD_DPAD_RIGHT:
                break;
            case FtcGamePad.GAMEPAD_DPAD_UP:
                break;
            case FtcGamePad.GAMEPAD_LBUMPER:
                if(pressed) {
                    turn90(TurnDirection.COUNTERCLOCKWISE, ticks);
                }
                break;
            case FtcGamePad.GAMEPAD_RBUMPER:
                if(pressed) {
                    turn90(TurnDirection.CLOCKWISE, ticks);
                }

                //this.robotDrive.setIsReverse(!this.robotDrive.getIsReverse());
                break;
            case FtcGamePad.GAMEPAD_START:
                robot.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
        }
    }

    @Override
    protected void handleOperatorGamepad(FtcGamePad gamepad, int button, boolean pressed) {

        switch (button) {
            case FtcGamePad.GAMEPAD_A:
                liftDownPressed = pressed;
                break;
            case FtcGamePad.GAMEPAD_B:

                break;
            case FtcGamePad.GAMEPAD_X:
                break;
            case FtcGamePad.GAMEPAD_Y:
                liftUpPressed = pressed;

                break;
            case FtcGamePad.GAMEPAD_DPAD_DOWN:
                break;
            case FtcGamePad.GAMEPAD_DPAD_LEFT:
                break;
            case FtcGamePad.GAMEPAD_DPAD_RIGHT:
                break;
            case FtcGamePad.GAMEPAD_DPAD_UP:
                break;
            case FtcGamePad.GAMEPAD_LBUMPER:
                break;
            case FtcGamePad.GAMEPAD_RBUMPER:
                flipperButtonPressed = pressed;

                break;
            case FtcGamePad.GAMEPAD_START:
                break;
        }
    }
}
