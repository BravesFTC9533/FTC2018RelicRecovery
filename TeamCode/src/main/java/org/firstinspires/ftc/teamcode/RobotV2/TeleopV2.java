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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Autonomous9533;
import org.firstinspires.ftc.teamcode.ComplicatedMecanumDrive;

import org.firstinspires.ftc.teamcode.FtcGamePad;
import org.firstinspires.ftc.teamcode.IDrive;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SensorBNO055IMU;
import org.firstinspires.ftc.teamcode.SimpleMenu;

import java.util.ArrayList;
import java.util.Locale;



/**
 * Created by 9533 on 2/3/2018.
 */
@TeleOp (name = "TeleOpV2", group = "Test")
@Disabled
public class TeleopV2 extends LinearOpMode  implements FtcGamePad.ButtonHandler {

    RobotV2 robot;
    protected FtcGamePad driverGamepad;
    protected FtcGamePad operatorGamepad;
    IDrive drive;

    IDrive complicatedDrive;
    IDrive mecanumDrive;

    PIDCoefficients pidOrigLeft;
    PIDCoefficients pidModified;

    public static double NEW_P = 10.0;
    public static double NEW_I = 10.0;
    public static double NEW_D = 3.0;
    public static SimpleMenu menu = new SimpleMenu("Autonomous Menu");




    @Override
    public void runOpMode() throws InterruptedException {

        driverGamepad = new FtcGamePad("DriverGamepad", gamepad1, this);
        operatorGamepad = new FtcGamePad("OperatorGamepad", gamepad2, this);

        driverGamepad.setYInverted(true);

        robot = new RobotV2(hardwareMap);

        complicatedDrive = new ComplicatedMecanumDrive_B(robot, driverGamepad);
        mecanumDrive = new MecanumDrive(robot, driverGamepad);


        robot.updatePID(NEW_P, NEW_I, NEW_D);

        robot.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);


        drive = complicatedDrive;

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


    Orientation angles;
    ComplicatedMecanumDrive_B.DriveModes driveModes;
    DcMotor.RunMode _currentRunMode;
    ArrayList<Integer> positions;

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            _currentRunMode = robot.GetMode();
            if(drive.getClass() == ComplicatedMecanumDrive_B.class) {
                driveModes = ((ComplicatedMecanumDrive_B) drive).getDriveMode();

            }
            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            positions = robot.getPositions();
        }
        });

        telemetry.addLine()
                .addData("Drive Type", new Func<String>() {
                    @Override public String value() {
                        return drive.getClass().getName();
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

        telemetry.addLine()
                .addData("Drive Mode", new Func<String>() {
                    @Override public String value() {
                        if(driveModes == ComplicatedMecanumDrive_B.DriveModes.FIELD) {
                            return "Field";
                        } else {
                            return "Robot";
                        }
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


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    @Override
    public void gamepadButtonEvent(FtcGamePad gamepad, int button, boolean pressed) {
        if(gamepad == driverGamepad) {

            switch (button)
            {
                case FtcGamePad.GAMEPAD_A: //counter clockwise
                    if(pressed) {

                        if(drive.getClass() == ComplicatedMecanumDrive_B.class) {
                            ComplicatedMecanumDrive_B.DriveModes mode = ((ComplicatedMecanumDrive_B) this.drive).getDriveMode();
                            if (mode == ComplicatedMecanumDrive_B.DriveModes.FIELD) {
                                ((ComplicatedMecanumDrive_B) this.drive).setDriveMode(ComplicatedMecanumDrive_B.DriveModes.ROBOT);
                            } else {
                                ((ComplicatedMecanumDrive_B) this.drive).setDriveMode(ComplicatedMecanumDrive_B.DriveModes.FIELD);
                            }
                        }
                    }
                    break;

                case FtcGamePad.GAMEPAD_B: //clockwise
                    if(pressed) {

                        if(drive.getClass() == MecanumDrive.class ){
                            drive = complicatedDrive;
                        } else {
                            drive = mecanumDrive;
                        }
                    }
                    break;

                case FtcGamePad.GAMEPAD_X:
                    if(pressed){

                        robot.updatePID(NEW_P, NEW_I, NEW_D);
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

                        drive.setIsReverse(!drive.getIsReverse());
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
