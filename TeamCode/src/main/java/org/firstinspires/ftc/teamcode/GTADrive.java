package org.firstinspires.ftc.teamcode;

/**
 * Created by dmill on 11/4/2017.
 */




class GTADrive implements IDrive {

    private final FtcGamePad driverGamepad;
    private final Robot robot;

    boolean reverse = false;

    public GTADrive(Robot robot, FtcGamePad driveGamepad){
        this.driverGamepad = driveGamepad;
        this.robot = robot;
    }


    @Override
    public boolean getIsReverse() {
        return reverse;
    }

    @Override
    public void setIsReverse(boolean value) {
        reverse = value;
    }

    public void handle() {

        double speed = driverGamepad.getRightTrigger() - driverGamepad.getLeftTrigger();
        double direction = -driverGamepad.getLeftStickX();



//        if(robot.gamepad1.x) {
//            reverse = false;
//        } else if(robot.gamepad1.y) {
//            reverse = true;
//        }

        DriveDir driveDir = getDir(direction);

        if(Math.abs(speed) < 5 && driveDir != DriveDir.STRAIGHT) {
            drive(-direction, direction);
        } else {
            if (reverse) {
                drive(driveDir.getLeft(reverse) * speed, driveDir.getRight(reverse) * speed);
            } else {
                drive(driveDir.getRight(reverse) * speed, driveDir.getLeft(reverse) * speed);
            }
        }

    }

    private DriveDir getDir(double dir) {
        if(dir <= -0.7) {
            return DriveDir.HARD_LEFT;
        } else if(dir <= -0.4) {
            return DriveDir.MEDIUM_LEFT;
        } else if(dir <= -0.1) {
            return DriveDir.EASY_LEFT;
        } else if(dir >= 0.7) {
            return DriveDir.HARD_RIGHT;
        } else if(dir >= 0.4) {
            return DriveDir.MEDIUM_RIGHT;
        } else if(dir >= 0.1) {
            return DriveDir.EASY_RIGHT;
        } else {
            return DriveDir.STRAIGHT;
        }
    }

    public enum DriveDir {
        HARD_LEFT(-1, 1),
        MEDIUM_LEFT(0, 1),
        EASY_LEFT(0.5, 1),
        STRAIGHT(1, 1),
        EASY_RIGHT(1, 0.5),
        MEDIUM_RIGHT(1, 0),
        HARD_RIGHT(1, -1);

        private double left, right;

        DriveDir(double left, double right) {
            this.left = left;
            this.right = right;
        }

        public double getLeft(boolean reverse) {
            return reverse ? -left : left;
        }

        public double getRight(boolean reverse) {
            return reverse ? -right : right;
        }
    }

    private void left(double power) {
        try {
            robot.motorLeft.setPower(power);
            //robot.setPower(power);
//                frontLeft.setPower(power);
//                backLeft.setPower(power);
        } catch(Exception ex) {}
    }

    private void right(double power) {
        try {
            robot.motorRight.setPower(power);
//                frontRight.setPower(power);
//                backRight.setPower(power);
        } catch(Exception ex) {}
    }

    public void drive(double left, double right) {
        left(left);
        right(right);
    }

    public void drive(double speed, DriveDir driveDir) {
        if(reverse) {
            drive(driveDir.getLeft(reverse) * speed, driveDir.getRight(reverse) * speed);
        } else {
            drive(driveDir.getRight(reverse) * speed, driveDir.getLeft(reverse) * speed);
        }
    }

    public void stop() {
        drive(0, 0);
    }


    int base = 0;
    public void resetEncoder() {
        base = robot.motorRight.getCurrentPosition();
    }

    public int getEncoder() {
        return robot.motorRight.getCurrentPosition() - base;
    }
}



class GTADrive2 implements IDrive {

    private final FtcGamePad driverGamepad;
    private final Robot robot;

    boolean reverse = false;

    public GTADrive2(Robot robot, FtcGamePad driveGamepad){
        this.driverGamepad = driveGamepad;
        this.robot = robot;
    }


    @Override
    public boolean getIsReverse() {
        return reverse;
    }

    @Override
    public void setIsReverse(boolean value) {
        reverse = value;
    }

    public void handle() {

        double speed = driverGamepad.getRightTrigger() - driverGamepad.getLeftTrigger();
        double direction = -driverGamepad.getLeftStickX();




        double leftDirection = -direction;
        double rightDirection = direction;


        double leftSpeed = leftDirection + speed;
        double rightSpeed = rightDirection + speed;

        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));

        double normalizationNumber = 1;
        if(max > 1) {
            normalizationNumber = 1 / max;
        }


        double finalLeft = leftSpeed * normalizationNumber;
        double finalRight = rightSpeed * normalizationNumber;



        if(reverse){
            drive(-finalRight, -finalLeft);
        } else {
            drive(finalLeft, finalRight);
        }


    }

    private DriveDir getDir(double dir) {
        if(dir <= -0.7) {
            return DriveDir.HARD_LEFT;
        } else if(dir <= -0.4) {
            return DriveDir.MEDIUM_LEFT;
        } else if(dir <= -0.1) {
            return DriveDir.EASY_LEFT;
        } else if(dir >= 0.7) {
            return DriveDir.HARD_RIGHT;
        } else if(dir >= 0.4) {
            return DriveDir.MEDIUM_RIGHT;
        } else if(dir >= 0.1) {
            return DriveDir.EASY_RIGHT;
        } else {
            return DriveDir.STRAIGHT;
        }
    }

    public enum DriveDir {
        HARD_LEFT(-1, 1),
        MEDIUM_LEFT(0, 1),
        EASY_LEFT(0.5, 1),
        STRAIGHT(1, 1),
        EASY_RIGHT(1, 0.5),
        MEDIUM_RIGHT(1, 0),
        HARD_RIGHT(1, -1);

        private double left, right;

        DriveDir(double left, double right) {
            this.left = left;
            this.right = right;
        }

        public double getLeft(boolean reverse) {
            return reverse ? -left : left;
        }

        public double getRight(boolean reverse) {
            return reverse ? -right : right;
        }
    }

    private void left(double power) {
        try {
            robot.motorLeft.setPower(power);
            //robot.setPower(power);
//                frontLeft.setPower(power);
//                backLeft.setPower(power);
        } catch(Exception ex) {}
    }

    private void right(double power) {
        try {
            robot.motorRight.setPower(power);
//                frontRight.setPower(power);
//                backRight.setPower(power);
        } catch(Exception ex) {}
    }

    public void drive(double left, double right) {
        left(left);
        right(right);
    }

    public void drive(double speed, DriveDir driveDir) {
        if(reverse) {
            drive(driveDir.getLeft(reverse) * speed, driveDir.getRight(reverse) * speed);
        } else {
            drive(driveDir.getRight(reverse) * speed, driveDir.getLeft(reverse) * speed);
        }
    }

    public void stop() {
        drive(0, 0);
    }


    int base = 0;
    public void resetEncoder() {
        base = robot.motorRight.getCurrentPosition();
    }

    public int getEncoder() {
        return robot.motorRight.getCurrentPosition() - base;
    }
}