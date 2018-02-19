package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.teamcode.*;

/**
 * Created by dmill on 2/17/2018.
 */
@Autonomous(name = "Autonomous V2", group = "Competition")
public class AutonomousV2_9533 extends LinearOpModeV2_9533 {


    private static final double cryptoBoxWidth = 7.5;
    private static final long pauseTimeBetweenSteps = 100;

    VuforiaHelper vuforiaHelper = null;
    ConfigV2 config = null;
    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;


    public static double NEW_P = 10.0;
    public static double NEW_I = 10.0;
    public static double NEW_D = 1.0;

    public static SimpleMenu menu = new SimpleMenu("Autonomous Menu");


    @Override
    protected IDrive getRobotDrive(RobotV2 robot, FtcGamePad gamePad) {
        return new ComplicatedMecanumDrive_B(robot, gamePad);
    }

    @Override
    protected void handleDriverGamepad(FtcGamePad gamepad, int button, boolean pressed) {

    }

    @Override
    protected void handleOperatorGamepad(FtcGamePad gamepad, int button, boolean pressed) {

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        menu.clearOptions();
        menu.addOption("P", 40, 0, 0.01, NEW_P);
        menu.addOption("I", 40, 0, 0.01, NEW_I);
        menu.addOption("D", 40, 0, 0.01, NEW_D);
        menu.setGamepad(gamepad1);
        menu.setTelemetry(telemetry);

        robot.updatePID(NEW_P, NEW_I, NEW_D);
        //config = ConfigV2.deserialize();

//        telemetry.addData("Status", "Initializing Vuforia");
//        telemetry.update();

//        vuforiaHelper = new VuforiaHelper();
//        vuforiaHelper.initVuforia(hardwareMap);
//
//        while(isStarted() == false) {
//            // just loop on vumark so we have it ready for start
//            preStart();
//            sleep(500);
//        }


        while(isStarted() == false) {
            menu.displayMenu();

            NEW_P = Double.parseDouble(menu.getCurrentChoiceOf("P"));
            NEW_I = Double.parseDouble(menu.getCurrentChoiceOf("I"));
            NEW_D = Double.parseDouble(menu.getCurrentChoiceOf("D"));

        }


//        telemetry.addData("Status", "Ready to start");
//        telemetry.update();
        waitForStart();

        //jewel time


        encoderDrive_straight(0.2, 18, 3);
        encoderDrive_straight(0.5, 10, 2);

        turn90(TurnDirection.COUNTERCLOCKWISE, 193);

        encoderDrive_straight(0.5, 10, 3);
        encoderDrive_straight(0.75, -25, 5);
        encoderDrive_straight(0.5, 25, 5);


        robot.stop();
    }

    void preStart() {
        vuMark = vuforiaHelper.detectVuMark();

        if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
            telemetry.addData("VuMark", "%s visible", vuMark);
        } else {
            telemetry.addData("VuMark", "not visible");
        }
    }




    void runAutonomous() {

        //read pictograph
        //knock off jewel
        //drive to cryptobox
        //drop off block
            //optional: get more blocks
            //drop off more blocks
        //park - get closer to blocks in center


    }

    void knockOffJewel() {
        //TODO: do this
    }


    void driveToCryptoBox() {




    }

    void driveToCryptoBoxRedFront() {
        //backwards
    }

    void driveToCryptoBoxBlueFront() {
        //forwards
        //config.distanceToCryptoBoxInchesFrontBlue




    }






    void readPictograph() {

        // read pictograph

//        updateStep("Detecting VuMark");
//        vuMark = detectVuMark(2.0);
//
//        if (vuMark == RelicRecoveryVuMark.CENTER) {
//
//            distanceToDrive += cryptoBoxWidth;
//
//        } else {
//
//            if (config.color == Config.Colors.RED) {
//
//                if (vuMark == RelicRecoveryVuMark.LEFT) {
//                    distanceToDrive += cryptoBoxWidth * 2;
//                }
//
//            } else {
//
//                if (vuMark == RelicRecoveryVuMark.RIGHT) {
//                    distanceToDrive += cryptoBoxWidth * 2;
//                }
//            }
//        }
//        updateStep("Finished detecting VuMark");

    }

    RelicRecoveryVuMark detectVuMark(double timeout) {
        ElapsedTime runtime = new ElapsedTime();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        while (opModeIsActive() && runtime.seconds() < timeout) {

            /**
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            vuMark = vuforiaHelper.detectVuMark();


            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                break;
            }
            else {
                telemetry.addData("VuMark", "not visible");

            }

            telemetry.update();
        }//while loop
        return vuMark;
    }

}
