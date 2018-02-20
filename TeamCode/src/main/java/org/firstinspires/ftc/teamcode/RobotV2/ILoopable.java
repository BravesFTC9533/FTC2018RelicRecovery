package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by 9533 on 2/19/2018.
 */

public interface ILoopable {
    void loop(ElapsedTime runTime);
    void stop();
}
