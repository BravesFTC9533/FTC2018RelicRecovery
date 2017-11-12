package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by dmill on 10/29/2017.
 */

@TeleOp(name = "TeleOp", group = "Competition")
public class Teleop9533 extends LinearOpMode implements FtcGamePad.ButtonHandler {


    private FtcGamePad driverGamepad;
    private FtcGamePad operatorGamepad;
    private Robot robot;

    private IDrive robotDrive;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);
        driverGamepad = new FtcGamePad("DriverGamepad", gamepad1, this);
        operatorGamepad = new FtcGamePad("OperatorGamepad", gamepad2, this);

        robotDrive = new GTADrive(robot, driverGamepad);


        telemetry.addData("Waiting for start..", "");
        telemetry.update();
        robot.GrabberStart();
        robot.retractColorArm();
        waitForStart();

        while(opModeIsActive()){

            driverGamepad.update();
            operatorGamepad.update();

            robotDrive.handle();

            robot.handleLiftMotor(gamepad2);


            telemetry.addData("Grabber L", "%.2f", robot.blockGrabberLeft.getPosition());
            telemetry.addData("Grabber R", "%.2f", robot.blockGrabberRight.getPosition());
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
            case FtcGamePad.GAMEPAD_A:

                break;

            case FtcGamePad.GAMEPAD_B:

                break;

            case FtcGamePad.GAMEPAD_X:
                if(pressed) {
                    //wind things up
                }
                break;

            case FtcGamePad.GAMEPAD_Y:
                break;

            case FtcGamePad.GAMEPAD_LBUMPER:
                //drivePowerScale = pressed? 0.5: 1.0;
                break;

            case FtcGamePad.GAMEPAD_RBUMPER:

                robotDrive.setIsReverse(!robotDrive.getIsReverse());
                break;
        }


    }

    private void handleOperatorGamepad(FtcGamePad gamepad, int button, boolean pressed) {


        switch (button)
        {
//            case FtcGamePad.GAMEPAD_A:
//
//
//
//                if(pressed){
//                    robot.GrabberLiftLower();
//                } else {
//                    robot.GrabberLiftStop();
//                }
//
//                break;

            case FtcGamePad.GAMEPAD_B:
                break;

            case FtcGamePad.GAMEPAD_X:
                break;

//            case FtcGamePad.GAMEPAD_Y:
//                if(pressed){
//                    robot.GrabberLiftRaise();
//                } else {
//                    robot.GrabberLiftStop();
//                }
//                break;

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



}
