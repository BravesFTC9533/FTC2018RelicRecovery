package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.lang.reflect.Array;
import java.util.ArrayList;

/**
 * Created by dmill on 10/28/2017.
 */

@Autonomous(name = "Autonomous Config", group = "Competition")
public class AutonomousConfig extends LinearOpMode {

    public static SimpleMenu menu = new SimpleMenu("Autonomous Menu");
    Config config = null;



    @Override
    public void runOpMode() {

        config = new Config(hardwareMap.appContext);

        //set some defaults..
        config.Park = true;
        config.JewelKnockOff = true;
        config.CryptoBox = true;
        config.color = Config.Colors.RED;
        config.position = Config.Positions.FRONT;

        config.Read();

        menu.clearOptions();

        menu.addOption("Team", Config.Colors.class, config.color);
        menu.addOption("Position", Config.Positions.class, config.position);
        menu.addBooleanOption("Park", config.Park);
        menu.addBooleanOption("Jewel Knockoff", config.JewelKnockOff);
        menu.addBooleanOption("Place Cryptoblock", config.CryptoBox);


        menu.addOption("Front Red Distance", 40, 10, 0.5, config.distanceToCryptoBoxInchesFrontRed);
        menu.addOption("Front Blue Distance", 40, 10, 0.5, config.distanceToCryptoBoxInchesFrontBlue);

        menu.addOption("Back Red Balance Board Distance", 40, 10, 0.5, config.distanceToDriveOffBalanceBoardBackRed);
        menu.addOption("Back Red Distance", 20, 10, 0.5, config.distanceToCryptoBoxInchesBackRed);

        menu.addOption("Back Blue Balance Board Distance", 40, 10, 0.5, config.distanceToDriveOffBalanceBoardBackBlue);
        menu.addOption("Back Blue Distance", 20, 10, 0.5, config.distanceToCryptoBoxInchesBackBlue);

        menu.addOption("Speed", 1, 0, 0.05, config.speed);
        menu.addOption("Delay Start", 30, 0, 0.1, config.delayStart);
        menu.addOption("Version", 100, 0, 1, config.version);

        menu.setGamepad(gamepad1);
        menu.setTelemetry(telemetry);

        waitForStart();

        while (opModeIsActive()) {

            menu.displayMenu();

            config.distanceToCryptoBoxInchesFrontRed        = Double.parseDouble(menu.getCurrentChoiceOf("Front Red Distance" ));
            config.distanceToCryptoBoxInchesFrontBlue       = Double.parseDouble(menu.getCurrentChoiceOf("Front Blue Distance"));
            config.distanceToCryptoBoxInchesBackRed         = Double.parseDouble(menu.getCurrentChoiceOf("Back Red Distance"));
            config.distanceToCryptoBoxInchesBackBlue        = Double.parseDouble(menu.getCurrentChoiceOf("Back Blue Distance"));
            config.distanceToDriveOffBalanceBoardBackBlue   = Double.parseDouble(menu.getCurrentChoiceOf("Back Blue Balance Board Distance"));
            config.distanceToDriveOffBalanceBoardBackRed    = Double.parseDouble(menu.getCurrentChoiceOf("Back Red Balance Board Distance"));
            config.delayStart                               = Double.parseDouble(menu.getCurrentChoiceOf("Delay Start"));
            config.speed                                    = Double.parseDouble(menu.getCurrentChoiceOf("Speed"));

            config.version                                  = Double.parseDouble(menu.getCurrentChoiceOf("Version"));
            switch (menu.getCurrentChoiceOf("Team")) {
                case "RED":
                    config.color = Config.Colors.RED;

                    //set red team
                    break;
                case "BLUE":
                    config.color = Config.Colors.BLUE;
                    //set blue team
                    break;
            }

            switch (menu.getCurrentChoiceOf("Position")) {
                case "FRONT":
                    config.position = Config.Positions.FRONT;
                    break;
                case "BACK":
                    config.position = Config.Positions.BACK;
                    break;
            }

            switch (menu.getCurrentChoiceOf("Park")) {
                case "YES":
                    config.Park = true;
                    break;
                case "NO":
                    config.Park = false;
                    break;
            }

            switch (menu.getCurrentChoiceOf("Jewel Knockoff")) {
                case "YES":
                    config.JewelKnockOff = true;
                    break;
                case "NO":
                    config.JewelKnockOff = false;
                    break;
            }

            switch (menu.getCurrentChoiceOf("Place Cryptoblock"))           //not really used?
            {
                case "YES":
                    config.CryptoBox = true;

                    break;
                case "NO":
                    config.CryptoBox = false;

                    break;
            }

//            switch (menu.getCurrentChoiceOf("Delay Start?")) {
//                case "YES":
//                    delayStartTime = 15;
//                    break;
//                case "NO":
//                    delayStartTime = 0;
//                    break;
//            }
            sleep(50);
        }

        config.Write();


    }
}
