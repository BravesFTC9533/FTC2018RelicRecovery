package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by dmill on 10/29/2017.
 */

@TeleOp(name = "TeleOp", group = "Competition")
public class Teleop9533 extends LinearOpMode implements FtcGamePad.ButtonHandler {


    private FtcGamePad driverGamepad;
    private FtcGamePad operatorGamepad;
    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);
        driverGamepad = new FtcGamePad("DriverGamepad", gamepad1, this);
        operatorGamepad = new FtcGamePad("OperatorGamepad", gamepad2, this);


        telemetry.addData("Waiting for start..", "");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){



            
            driverGamepad.update();
            operatorGamepad.update();
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

    }
    private void handleOperatorGamepad(FtcGamePad gamepad, int button, boolean pressed) {

    }
}
