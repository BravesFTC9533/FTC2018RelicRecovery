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

    double currentSpeed = 0;
    public IntakeMotor(RobotV2 robot, FtcGamePad operaterGamepad){
        this.robot = robot;
        this.intakeMotorLeft = robot.intakeMotorLeft;
        this.intakeMotorRight = robot.intakeMotorRight;
        this.operatorGamepad = operaterGamepad;
    }

<<<<<<< HEAD
    double pullSpeed = operatorGamepad.getRightTrigger();
    double spitSpeed = operatorGamepad.getLeftTrigger();
=======
    double pullSpeed;
    double spitSpeed;
>>>>>>> origin/master

    @Override
    public void loop(ElapsedTime runTime) {

        currentSpeed = operatorGamepad.getRightTrigger() - operatorGamepad.getLeftTrigger();
        setPower(currentSpeed);

    }

    @Override
    public void stop() {
        stopIntake();
    }

    public double getCurrentSpeed() {
        return currentSpeed;
    }
    public void setPower(double power){
        intakeMotorLeft.setPower(power);
        intakeMotorRight.setPower(power);
    }

    public int getLeftTicks() {
        return intakeMotorLeft.getCurrentPosition();
    }
    public int getRightTicks() {
        return intakeMotorRight.getCurrentPosition();
    }


    public void stopIntake(){
        intakeMotorRight.setPower(0);
        intakeMotorLeft.setPower(0);
    }
}
