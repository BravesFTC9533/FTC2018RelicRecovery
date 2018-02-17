package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Autonomous9533;
import org.firstinspires.ftc.teamcode.ComplicatedMecanumDrive;

import org.firstinspires.ftc.teamcode.FtcGamePad;
import org.firstinspires.ftc.teamcode.IDrive;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SensorBNO055IMU;
import org.firstinspires.ftc.teamcode.SimpleMenu;


/**
 * Created by 9533 on 2/3/2018.
 */
@TeleOp (name = "TeleOpV2", group = "Test")
//@Disabled
public class TeleopV2 extends LinearOpMode  implements FtcGamePad.ButtonHandler {

    RobotV2 robot;
    protected FtcGamePad driverGamepad;
    protected FtcGamePad operatorGamepad;
    IDrive drive;
    PIDCoefficients pidOrigLeft;
    PIDCoefficients pidModified;

    public static double NEW_P = 10;
    public static double NEW_I = 3.0;
    public static double NEW_D = 0.0;
    public static SimpleMenu menu = new SimpleMenu("Autonomous Menu");



    @Override
    public void runOpMode() throws InterruptedException {

        driverGamepad = new FtcGamePad("DriverGamepad", gamepad1, this);
        operatorGamepad = new FtcGamePad("OperatorGamepad", gamepad2, this);

        robot = new RobotV2(hardwareMap);
        //drive = new ComplicatedMecanumDrive(robot, driverGamepad);
        drive = new ComplicatedMecanumDrive_B(robot, driverGamepad);

        robot.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);



        menu.clearOptions();
        menu.addOption("P", 40, 0, 0.01, NEW_P);
        menu.addOption("I", 40, 0, 0.01, NEW_I);
        menu.addOption("D", 40, 0, 0.01, NEW_D);
        menu.setGamepad(gamepad1);
        menu.setTelemetry(telemetry);

        pidModified = robot.GetPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        composeTelemetry();

        while(opModeIsActive()){

            pidModified = robot.GetPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            driverGamepad.update();
            drive.handle();
            

            menu.displayMenu();

            NEW_P = Double.parseDouble(menu.getCurrentChoiceOf("P"));
            NEW_I = Double.parseDouble(menu.getCurrentChoiceOf("I"));
            NEW_D = Double.parseDouble(menu.getCurrentChoiceOf("D"));


        }
    }


    DcMotor.RunMode _currentRunMode;

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            _currentRunMode = robot.GetMode();
        }
        });

        telemetry.addLine()
                .addData("RunMode", new Func<String>() {
                    @Override public String value() {
                        if(_currentRunMode == DcMotor.RunMode.RUN_USING_ENCODER) {
                            return "Encoder";
                        } else {
                            return "Normal";
                        }
                    }
                });

        telemetry.addLine("P,I,D (modified)")
                .addData("P", new Func<String>() {
                    @Override public String value() {
                        return String.format("%.04f", pidModified.p);
                    }

                }).addData("I", new Func<String>() {
                    @Override public String value() {
                        return String.format("%.04f", pidModified.i);
                    }

                }).addData("D", new Func<String>() {
                    @Override public String value() {
                        return String.format("%.04f", pidModified.d);
                    }

                });


    }

    @Override
    public void gamepadButtonEvent(FtcGamePad gamepad, int button, boolean pressed) {
        if(gamepad == driverGamepad) {

            switch (button)
            {
                case FtcGamePad.GAMEPAD_A: //counter clockwise
                    if(pressed) {

                    }
                    break;

                case FtcGamePad.GAMEPAD_B: //clockwise
                    if(pressed) {

                    }
                    break;

                case FtcGamePad.GAMEPAD_X:
                    if(pressed){

                        robot.SetPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, NEW_P, NEW_I, NEW_D);

                    }
                    break;

                case FtcGamePad.GAMEPAD_Y:
                    if(pressed){
                        DcMotor.RunMode currentMode = robot.GetMode();
                        if(currentMode == DcMotor.RunMode.RUN_USING_ENCODER) {
                            currentMode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
                        } else {
                            currentMode = DcMotor.RunMode.RUN_USING_ENCODER;
                        }
                        robot.SetMode(currentMode);
                    }
                    break;

                case FtcGamePad.GAMEPAD_LBUMPER:
                    //drivePowerScale = pressed? 0.5: 1.0;
                    break;

                case FtcGamePad.GAMEPAD_RBUMPER:

                    if(pressed) {

                    }

                    break;
                case FtcGamePad.GAMEPAD_START:
                    if(pressed) {

                    }
                    break;
                case FtcGamePad.GAMEPAD_BACK:

                    break;

            }

        }
    }
}
