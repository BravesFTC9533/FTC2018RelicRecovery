package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Created by 9533 on 2/3/2018.
 */
@TeleOp (name = "TeleOpV2", group = "Test")
public class TeleopV2 extends LinearOpMode {
RobotV2 robot;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotV2(hardwareMap);
        waitForStart();





        while(opModeIsActive()){
            if(gamepad1.a){
                robot.driveForward();
            } else {
                robot.stop();
            }
        }
    }
}
