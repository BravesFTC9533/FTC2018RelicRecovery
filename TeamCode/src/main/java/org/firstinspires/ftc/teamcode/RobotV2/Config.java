package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Created by doug on 2/16/2018.
 */

public class Config implements Cloneable {

    public enum Colors {
        RED,
        BLUE


    }

    public enum Positions {
        FRONT,
        BACK

    }



    public Colors color;
    public Positions position;



    public boolean Park, JewelKnockOff, CryptoBox;
    public double distanceToCryptoBoxInchesFrontRed, distanceToCryptoBoxInchesFrontBlue;
    public double distanceToDriveOffBalanceBoardBackBlue,distanceToDriveOffBalanceBoardBackRed;
    public double distanceToCryptoBoxInchesBackRed,distanceToCryptoBoxInchesBackBlue;
    public double delayStart,speed,version;


    public String serialize() {
        return SimpleGson.getInstance().toJson(this);
    }
    public static Config deserialize(String data) {
        return SimpleGson.getInstance().fromJson(data, Config.class);
    }

    public void save() {
        String filename = "config2.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, this.serialize());
    }

    public Config clone()
    {
        try {
            Config result = (Config)super.clone();
            return result;
        }
        catch (CloneNotSupportedException e)
        {
            throw new RuntimeException("internal error: Config can't be cloned");
        }
    }
}
