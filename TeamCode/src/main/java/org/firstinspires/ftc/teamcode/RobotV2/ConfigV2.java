package org.firstinspires.ftc.teamcode.RobotV2;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import android.content.Context;
import java.io.File;
import java.io.FileInputStream;

/**
 * Created by doug on 2/16/2018.
 */

public class ConfigV2 implements Cloneable {

    public enum Colors {
        RED,
        BLUE
    }

    public enum Positions {
        FRONT,
        BACK
    }

    static final String filename = "config2.json";

    public Colors color;
    public Positions position;

    public boolean Park, JewelKnockOff, CryptoBox;
    public double distanceToCryptoBoxInchesFrontRed, distanceToCryptoBoxInchesFrontBlue;
    public double distanceToDriveOffBalanceBoardBackBlue,distanceToDriveOffBalanceBoardBackRed;
    public double distanceToCryptoBoxInchesBackRed,distanceToCryptoBoxInchesBackBlue;
    public double delayStart,speed,version;

    Context context = null;



    public String serialize() {
        return SimpleGson.getInstance().toJson(this);
    }


    public static ConfigV2 deserialize() {
        File file = AppUtil.getInstance().getSettingsFile(filename);
        String json = ReadWriteFile.readFile(file);
        return SimpleGson.getInstance().fromJson(json, ConfigV2.class);
    }

    public static ConfigV2 deserialize(String data) {
        return SimpleGson.getInstance().fromJson(data, ConfigV2.class);
    }



    public void save() {

        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, this.serialize());
    }

    public ConfigV2 clone()
    {
        try {
            ConfigV2 result = (ConfigV2)super.clone();
            return result;
        }
        catch (CloneNotSupportedException e)
        {
            throw new RuntimeException("internal error: Config can't be cloned");
        }
    }
}
