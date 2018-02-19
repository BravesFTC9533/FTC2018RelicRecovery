package org.firstinspires.ftc.teamcode;

/**
 * Created by 9533 on 2/3/2018.
 */

public interface IDrive {

    boolean getIsReverse();
    void setIsReverse(boolean value);
    void handle();

    void drive(double ly, double lx, double rx);
}
