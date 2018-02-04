package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.FtcGamePad;
import org.firstinspires.ftc.teamcode.IDrive;
import org.firstinspires.ftc.teamcode.MecanumDrive;


/**
 * Created by 9533 on 2/3/2018.
 */
@TeleOp (name = "TeleOpV2", group = "Test")
public class TeleopV2 extends LinearOpMode  implements FtcGamePad.ButtonHandler {

    RobotV2 robot;
    protected FtcGamePad driverGamepad;
    protected FtcGamePad operatorGamepad;
    IDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {

        driverGamepad = new FtcGamePad("DriverGamepad", gamepad1, this);
        operatorGamepad = new FtcGamePad("OperatorGamepad", gamepad2, this);


        robot = new RobotV2(hardwareMap);
        drive = new MecanumDrive(robot, driverGamepad);


        waitForStart();


        while(opModeIsActive()){


            drive.handle();

        }
    }

    @Override
    public void gamepadButtonEvent(FtcGamePad gamepad, int button, boolean pressed) {

    }
}
