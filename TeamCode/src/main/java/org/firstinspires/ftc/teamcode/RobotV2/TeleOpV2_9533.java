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

    BlockLift blockLift = null;
    BlockTray blockTray = null;
    IntakeMotor intakeMotor = null;
    ElapsedTime runtime = new ElapsedTime();

    //<editor-fold desc="Telemetry variables">
    Orientation angles;
    ComplicatedMecanumDrive_B.DriveModes currentDriveMode;
    boolean usingEncoder = true;
    //</editor-fold>


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        blockLift = new BlockLift(robot);
        blockTray = new BlockTray(robot);

        intakeMotor = new IntakeMotor(robot, operatorGamepad);

        telemetry.addData("*** Driver Controls ***", "");
        telemetry.addData("[B] Toggle Field/Robot Orientation", "");
        telemetry.addData("[X] Toggle Normal/Reverse drive", "");
        telemetry.addData("[Y] Toggle Encoder drive", "");

        telemetry.addData("*** Operator Controls ***", "");
        telemetry.addData("[A] Lower lift", "");
        telemetry.addData("[Y] Raise lift", "");
        telemetry.addData("[RB] Open Block Tray", "");

        telemetry.update();

        waitForStart();


        ComposeTelemetry();


        runtime.reset();
        while(opModeIsActive()) {

            updateGamepads();
            robotDrive.handle();

            blockLift.loop(runtime);
            blockTray.loop(runtime);
            intakeMotor.loop(runtime);

            //TODO: do not allow intake to run while tray is open

            telemetry.update();

        }

    }


    private void ComposeTelemetry() {

        telemetry.addAction(new Runnable() { @Override public void run()
        {
            angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentDriveMode = ((ComplicatedMecanumDrive_B)robotDrive).getDriveMode();
            usingEncoder = robot.GetMode() == DcMotor.RunMode.RUN_USING_ENCODER;
        }
        });

        telemetry.addLine()
                .addData("Orientation", new Func<String>() {
                            @Override
                            public String value() {
                                return currentDriveMode.toString();
                            }
                        });
        telemetry.addLine()
                .addData("Drive Mode", new Func<String>() {
                    @Override
                    public String value() {
                        return robotDrive.getIsReverse() ? "Reverse" : "Normal";
                    }
                });
        telemetry.addLine()
                .addData("Using Encoders?", new Func<String>() {
                    @Override
                    public String value() {
                        return usingEncoder ? "Yes" : "No";
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
                if(pressed) {
                    ((ComplicatedMecanumDrive_B) robotDrive).toggleDriveMode();
                }
                break;
            case FtcGamePad.GAMEPAD_X:
                if(pressed) {
                    robotDrive.setIsReverse(!robotDrive.getIsReverse());
                }
                break;
            case FtcGamePad.GAMEPAD_Y:
                if(pressed) {
                    switch (robot.GetMode()){
                        case RUN_USING_ENCODER:
                            robot.SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            break;
                        case RUN_WITHOUT_ENCODER:
                            robot.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
                            break;
                    }
                }
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
                    //turn90(TurnDirection.COUNTERCLOCKWISE, ticks);
                }
                break;
            case FtcGamePad.GAMEPAD_RBUMPER:
                if(pressed) {
                    //turn90(TurnDirection.CLOCKWISE, ticks);
                }

                //this.robotDrive.setIsReverse(!this.robotDrive.getIsReverse());
                break;
            case FtcGamePad.GAMEPAD_START:
//                robot.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                robot.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
        }
    }

    @Override
    protected void handleOperatorGamepad(FtcGamePad gamepad, int button, boolean pressed) {

        switch (button) {
            case FtcGamePad.GAMEPAD_A:
                //liftDownPressed = pressed;
                blockLift.setLowerLiftButtonPressed(pressed);
                break;
            case FtcGamePad.GAMEPAD_B:


                break;
            case FtcGamePad.GAMEPAD_X:
                break;
            case FtcGamePad.GAMEPAD_Y:
                //liftUpPressed = pressed;
                blockLift.setRaiseLiftButtonPressed(pressed);

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
                //flipperButtonPressed = pressed;
                blockTray.setOpenTrayButtonPressed(pressed);
                break;
            case FtcGamePad.GAMEPAD_START:
                break;
        }
    }
}
