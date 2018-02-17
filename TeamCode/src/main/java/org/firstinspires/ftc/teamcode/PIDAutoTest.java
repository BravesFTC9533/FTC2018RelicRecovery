package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Created by dmill on 2/3/2018.
 */
@Autonomous(name = "PID Auto Test", group = "Testing")
//@Disabled
public class PIDAutoTest extends LinearOpMode9533 {




    public static double NEW_P = 10.0;
    public static double NEW_I = 10.0;
    public static double NEW_D = 1.0;
    public static double NEW_KP = 0.1;

    public static SimpleMenu menu = new SimpleMenu("Autonomous Menu");


    @Override
    public void runOpMode() throws InterruptedException {


        initialize();

        menu.clearOptions();
        menu.addOption("P", 1000, 0, 0.01, NEW_P);
        menu.addOption("I", 1000, 0, 0.01, NEW_I);
        menu.addOption("D", 1000, 0, 0.01, NEW_D);
        menu.addOption("Kp", 1000, 0, 0.01, NEW_KP);
        menu.setGamepad(gamepad1);
        menu.setTelemetry(telemetry);


        waitForStart();

        boolean startPressed = false;


        while(opModeIsActive()) {

            if (gamepad1.start && startPressed == false) {
                startPressed = true;

            }
            if(!gamepad1.start && startPressed) {
                startPressed = false;
                this.Kp = NEW_KP;
                updatePID();
                encoderDrive_straight(0.75, 48, 1000.0);
            }

            menu.displayMenu();
            NEW_P = Double.parseDouble(menu.getCurrentChoiceOf("P"));
            NEW_I = Double.parseDouble(menu.getCurrentChoiceOf("I"));
            NEW_D = Double.parseDouble(menu.getCurrentChoiceOf("D"));
            NEW_KP = Double.parseDouble(menu.getCurrentChoiceOf("Kp"));


            telemetry.update();
        }



    }

    public void DriveLoop() {





//
//        //encoderDriveBasic(0.5, 45, 45, 10.0);
//
//
//        for(int i = 0;i<2;i++) {
//
//            robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            sleep(100);
//            turn90(Autonomous9533.TurnDirection.CLOCKWISE);
//            sleep(1000);
//
//        }
//
//        encoderDriveBasic(0.5, 45, 45, 10.0);
    }




}
