package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FtcGamePad;

/**
 * Created by 9533 on 2/23/2018.
 */

public class IntakeMotor implements ILoopable {

    private final RobotV2 robot;

    private final DcMotor intakeMotorLeft;
    private final DcMotor intakeMotorRight;

    private FtcGamePad operatorGamepad = null;

    public IntakeMotor(RobotV2 robot, FtcGamePad operaterGamepad){
        this.robot = robot;
        this.intakeMotorLeft = robot.intakeMotorLeft;
        this.intakeMotorRight = robot.intakeMotorRight;
        this.operatorGamepad = operaterGamepad;
    }
    
    double pullSpeed = operatorGamepad.getRightTrigger();
    double spitSpeed = operatorGamepad.getLeftTrigger();

    @Override
    public void loop(ElapsedTime runTime) {
        if(operatorGamepad.getRightTrigger() > 0) {
            pullBlock();
        }
        if(operatorGamepad.getLeftTrigger() > 0){
            spitBlock();
        }
    }

    @Override
    public void stop() {
        stopIntake();
    }

    public void pullBlock(){
        intakeMotorLeft.setPower(pullSpeed);
        intakeMotorRight.setPower(pullSpeed);
    }

    public void spitBlock(){
        intakeMotorLeft.setPower(-spitSpeed);
        intakeMotorRight.setPower(-spitSpeed);
    }


    public void stopIntake(){
        intakeMotorRight.setPower(0);
        intakeMotorLeft.setPower(0);
    }
}
