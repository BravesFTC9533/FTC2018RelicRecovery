package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.RobotV2.RobotV2;


/**
 * Created by 9533 on 2/3/2018.
 */
@TeleOp (name = "TeleOpTest", group = "Test")
@Disabled
public class TeleopTest extends LinearOpMode9533  implements FtcGamePad.ButtonHandler {


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


        robot = new Robot(hardwareMap);
        robotDrive = new GTADrive(robot, driverGamepad);

        robot.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);



        menu.clearOptions();
        menu.addOption("P", 4000, 0, 0.01, NEW_P);
        menu.addOption("I", 4000, 0, 0.01, NEW_I);
        menu.addOption("D", 4000, 0, 0.01, NEW_D);
        menu.setGamepad(gamepad1);
        menu.setTelemetry(telemetry);

        pidModified = robot.GetPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        composeTelemetry();

        while(opModeIsActive()){

            pidModified = robot.GetPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

            driverGamepad.update();
            robotDrive.handle();

            menu.displayMenu();

            NEW_P = Double.parseDouble(menu.getCurrentChoiceOf("P"));
            NEW_I = Double.parseDouble(menu.getCurrentChoiceOf("I"));
            NEW_D = Double.parseDouble(menu.getCurrentChoiceOf("D"));


        }
    }


    void driveForward() {
        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int lpos = robot.motorLeft.getCurrentPosition();
        int rpos = robot.motorRight.getCurrentPosition();
        double inches = 48;
        int newLeftTarget = robot.motorLeft.getCurrentPosition() + (int)(inches * robot.REV_COUNTS_PER_INCH);
        int newRightTarget = robot.motorRight.getCurrentPosition() + (int)(inches * robot.REV_COUNTS_PER_INCH);
        robot.motorLeft.setTargetPosition(newLeftTarget);
        robot.motorRight.setTargetPosition(newRightTarget);

        while(opModeIsActive()){

            lpos = robot.motorLeft.getCurrentPosition();
            rpos = robot.motorRight.getCurrentPosition();

            if(Math.abs(newLeftTarget - lpos) < 50){
                robot.motorLeft.setPower(0);
            }
            if(Math.abs(newRightTarget - rpos) < 50){
                robot.motorRight.setPower(0);
            }

            robot.setPower(1, 1);

        }
        robot.stop();


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

                        encoderDrive(1, 36, 36, 5);
                    }
                    break;

                case FtcGamePad.GAMEPAD_B: //clockwise
                    if(pressed) {

                        turn90(Autonomous9533.TurnDirection.COUNTERCLOCKWISE);
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
