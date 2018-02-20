package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by 9533 on 2/19/2018.
 */

public class BlockLift implements ILoopable {

    private final RobotV2 robot;

    private final DcMotor liftMotor;

    private final DigitalChannel touchSensor;

    private boolean raiseLiftButtonPressed = false;
    private boolean lowerLiftButtonPressed = false;


    private final static double LIFT_SPEED = 1.0;



    public BlockLift(RobotV2 robot) {
        this.robot = robot;

        this.liftMotor = robot.motorLift;

        this.touchSensor = robot.touchSensor;
    }


    public void setRaiseLiftButtonPressed(boolean pressed) {
        this.raiseLiftButtonPressed = pressed;
    }
    public void setLowerLiftButtonPressed(boolean pressed){
        this.lowerLiftButtonPressed = pressed;
    }

    public void loop(ElapsedTime runTime) {
        if(lowerLiftButtonPressed){
            if(touchSensorPressed()) {
                stopLift();
            } else {
                lowerLift();
            }
        } else if(raiseLiftButtonPressed) {
            raiseLift();
        } else {
            stopLift();
        }

    }

    private boolean touchSensorPressed() {
        return  !(touchSensor.getState() == true);
    }

    private void raiseLift() {
        liftMotor.setPower(LIFT_SPEED);
    }
    private void lowerLift(){
        liftMotor.setPower(-LIFT_SPEED);
    }
    private void stopLift(){
        liftMotor.setPower(0);
    }


    public void stop() {
        stopLift();
    }

}
