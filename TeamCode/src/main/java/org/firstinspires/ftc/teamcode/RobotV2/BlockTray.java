package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 9533 on 2/19/2018.
 */

public class BlockTray implements ILoopable {

    //region Hardware
    private final Servo servoLeft;
    private final Servo servoRight;
    private final RobotV2 robot;
    //endregion

    private double lastMillis = 0;
    private final static double MIN_SERVO = 0.3;
    static final double servoDelta = 0.02;
    public static double servoDelayDelta = 1000/100;
    public double servoPosition = MIN_SERVO;
    private boolean openTrayButtonPressed = false;

    public BlockTray(RobotV2 robot) {
        this.robot = robot;
        this.servoLeft = robot.flipperServoLeft;
        this.servoRight = robot.flipperServoRight;

    }

    public void setOpenTrayButtonPressed(boolean pressed){
        this.openTrayButtonPressed = pressed;
    }
    public double getServoPosition(){
        return  servoPosition;
    }

    public void closeBlockTray() {
        servoPosition -= servoDelta;

        if(servoPosition < MIN_SERVO) {
            servoPosition = MIN_SERVO;
        }
        setBlockFlipperServoPosition(servoPosition);
    }
    public void openBlockTray() {
        servoPosition += servoDelta;
        if(servoPosition > 1) {
            servoPosition = 1;
        }
        setBlockFlipperServoPosition(servoPosition);
    }

    private void setBlockFlipperServoPosition(double position) {
        servoLeft.setPosition(position);
        servoRight.setPosition(position);
    }

    @Override
    public void loop(ElapsedTime runTime) {
        double currentMillis = runTime.milliseconds();

        if(openTrayButtonPressed) {
            boolean shouldOpen = lastMillis == 0 || currentMillis >= lastMillis + servoDelayDelta;
            if(shouldOpen) {
                openBlockTray();
                lastMillis = currentMillis;
            }
        } else {
            boolean shouldClose = lastMillis == 0 || currentMillis >= lastMillis + servoDelayDelta;
            if(shouldClose) {
                closeBlockTray();
                lastMillis = currentMillis;
            }
        }
    }

    @Override
    public void stop() {

    }
}