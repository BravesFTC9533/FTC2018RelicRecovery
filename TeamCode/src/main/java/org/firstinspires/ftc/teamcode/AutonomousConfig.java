package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

        config.Park = true;
        config.JewelKnockOff = true;
        config.CryptoBox = true;
        config.color = Config.Colors.RED;
        config.position = Config.Positions.FRONT;

        menu.clearOptions();

        menu.addOption("Team", new String[]{"RED", "BLUE"});
        menu.addOption("Position", new String[]{"FRONT", "BACK"});

        menu.addOption("Park", new String[]{"YES", "NO"});
        menu.addOption("Jewel Knockoff", new String[]{"YES", "NO"});
        menu.addOption("Place Cryptoblock", new String[]{"YES", "NO"});
        //menu.addOption("Delay Start", 30, 0, 0.1);

        menu.setGamepad(gamepad1);
        menu.setTelemetry(telemetry);

        waitForStart();

        while (opModeIsActive()) {

            menu.displayMenu();

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
        }

        config.Write();


    }
}
