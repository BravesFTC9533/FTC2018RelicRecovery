package org.firstinspires.ftc.teamcode;


import android.content.Context;
import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.InputStream;
import java.io.Reader;
import java.io.Writer;
import java.util.ArrayList;
import java.util.concurrent.Exchanger;

/**
 * Created by 9533 on 10/28/2017.
 */
//save & write methods
    //red & blue enums
    //front & back enums
    //booleans for park
public class Config {

    private static final String fileName = "config.json";

    private static final String PARKNAME = "park";
    private static final String JEWELNAME = "jewel";
    private static final String CRYPTONAME = "crypto";

    private static final String COLOR = "color";
    private static final String POSITION = "position";

    private static final String DISTANCETODRIVEOFFBALANCEBOARDBACKBLUE = "distanceToDriveOffBalanceBoardBackBlue";
    private static final String DISTANCETODRIVEOFFBALANCEBOARDBACKRED = "distanceToDriveOffBalanceBoardBackRed";

    private static final String DISTANCETOCRYPTOBOXINCHESBACKBLUE = "distanceToCryptoBoxInchesBackBlue";
    private static final String DISTANCETOCRYPTOBOXINCHESBACKRED = "distanceToCryptoBoxInchesBackRed";


    private static final String DISTANCETOCRYPTOBOXINCHESFRONTBLUE = "distanceToCryptoBoxInchesFrontBlue";
    private static final String DISTANCETOCRYPTOBOXINCHESFRONTRED = "distanceToCryptoBoxInchesFrontRed";

    private static final String DELAYSTART = "delayStart";
    private static final String SPEED = "speed";

    private static final String VERSION = "version";


    Context context;
    public Config(Context context) {
        this.context = context;

    }


    private <T> T tryGetValue(JSONObject json, String name, T defaultValue, Class<T> type) {
        if(type == null) throw new NullPointerException("type is null");

        if(json.has(name) == false) {
            return defaultValue;
        }
        try {

            String value = json.getString(name);

            if(type.equals(Integer.class)) {
                return  (T)Integer.valueOf(value);
            } else if(type.equals(Double.class)) {
                return  (T)Double.valueOf(value);
            } else if(type.equals(String.class)) {
                return  (T)String.valueOf(value);
            }
        }
        catch(NumberFormatException|NullPointerException|JSONException ex){

        }
        throw new IllegalArgumentException("Invalid class " + type);
    }

    public void Read(){

        try {
            File f = new File(context.getFilesDir(), fileName);
            if (f.exists()) {

                InputStream is = context.openFileInput(fileName);
                String jsonTxt = convertStreamToString(is);



                JSONObject json = new JSONObject(jsonTxt);

                String c = json.getString(COLOR);

                if(c.equals("RED")) {
                    this.color = Colors.RED;
                } else {
                    this.color = Colors.BLUE;
                }

                String p = json.getString("position");

                if(p.equals("FRONT")) {
                    this.position = Positions.FRONT;
                } else {
                    this.position = Positions.BACK;
                }

                this.JewelKnockOff = json.getBoolean(JEWELNAME);
                this.CryptoBox = json.getBoolean(CRYPTONAME);
                this.Park = json.getBoolean(PARKNAME);

                if(json.has(VERSION) && json.getInt(VERSION) != 0) {


                    this.distanceToCryptoBoxInchesFrontRed = tryGetValue(json, DISTANCETOCRYPTOBOXINCHESFRONTRED, 32.5, Double.class);
                    this.distanceToCryptoBoxInchesFrontBlue = tryGetValue(json, DISTANCETOCRYPTOBOXINCHESFRONTBLUE, 18.0, Double.class);


                    this.distanceToDriveOffBalanceBoardBackBlue = tryGetValue(json, DISTANCETODRIVEOFFBALANCEBOARDBACKBLUE, 26.0, Double.class);
                    this.distanceToDriveOffBalanceBoardBackRed = tryGetValue(json, DISTANCETODRIVEOFFBALANCEBOARDBACKRED,  32.0, Double.class);
                    this.distanceToCryptoBoxInchesBackRed = tryGetValue(json, DISTANCETOCRYPTOBOXINCHESBACKRED, 13.5, Double.class);
                    this.distanceToCryptoBoxInchesBackBlue = tryGetValue(json, DISTANCETOCRYPTOBOXINCHESBACKBLUE, 13.0, Double.class);

                    this.delayStart = json.getDouble(DELAYSTART);
                    this.speed = json.getDouble(SPEED);
                    this.version = json.getInt(VERSION);
//                    this.distanceToDriveOffBalanceBoardBackBlue = json.getDouble(DISTANCETODRIVEOFFBALANCEBOARDBACKBLUE);
//                    this.distanceToDriveOffBalanceBoardBackRed = json.getDouble(DISTANCETODRIVEOFFBALANCEBOARDBACKRED);
//
//                    this.distanceToCryptoBoxInchesFrontRed = json.getDouble(DISTANCETOCRYPTOBOXINCHESFRONTRED);
//                    this.distanceToCryptoBoxInchesFrontBlue = json.getDouble(DISTANCETOCRYPTOBOXINCHESFRONTBLUE);



                } else {
                    //give default values
                    this.distanceToCryptoBoxInchesFrontRed = 32.5;
                    this.distanceToCryptoBoxInchesFrontBlue = 18.0;


                    this.distanceToDriveOffBalanceBoardBackBlue = 26.0;
                    this.distanceToDriveOffBalanceBoardBackRed = 32.0;
                    this.distanceToCryptoBoxInchesBackRed = 13.5;
                    this.distanceToCryptoBoxInchesBackBlue = 13.0;

                    this.delayStart = 0.0;
                    this.speed = 0.5;

                    this.version = 1;
                }

            }
        } catch (Exception ex) {

        }
    }



    static String convertStreamToString(java.io.InputStream is) {
        java.util.Scanner s = new java.util.Scanner(is).useDelimiter("\\A");
        return s.hasNext() ? s.next() : "";
    }

    public void Write(){

        JSONObject obj = new JSONObject() ;

        try {
            obj.put(COLOR, color.toString());
            obj.put(POSITION, position.toString());
            obj.put(PARKNAME, Park);
            obj.put(JEWELNAME, JewelKnockOff);
            obj.put(CRYPTONAME, CryptoBox);

            obj.put(DISTANCETOCRYPTOBOXINCHESBACKBLUE, this.distanceToCryptoBoxInchesBackBlue);
            obj.put(DISTANCETOCRYPTOBOXINCHESBACKRED, this.distanceToCryptoBoxInchesBackRed);
            obj.put(DISTANCETODRIVEOFFBALANCEBOARDBACKRED, this.distanceToDriveOffBalanceBoardBackRed);
            obj.put(DISTANCETODRIVEOFFBALANCEBOARDBACKBLUE, this.distanceToDriveOffBalanceBoardBackBlue);
            obj.put(DISTANCETOCRYPTOBOXINCHESFRONTBLUE, this.distanceToCryptoBoxInchesFrontBlue);
            obj.put(DISTANCETOCRYPTOBOXINCHESFRONTRED, this.distanceToCryptoBoxInchesFrontRed);

            obj.put(DELAYSTART, this.delayStart);
            obj.put(SPEED, this.speed);

            obj.put(VERSION, this.version);
        } catch (JSONException e) {
            e.printStackTrace();
        }


        try {
            Writer output = null;

            File file = new File(context.getFilesDir(), fileName);
            output = new BufferedWriter(new FileWriter(file));
            output.write(obj.toString());
            output.close();
        } catch(Exception ex) {

        }


    }




    public enum Colors {
        RED,
        BLUE


    }

    public enum Positions {
        FRONT,
        BACK

    }

    public boolean Park;
    public boolean JewelKnockOff;
    public boolean CryptoBox;

    public Colors color;
    public Positions position;




    public double distanceToCryptoBoxInchesFrontRed;
    public double distanceToCryptoBoxInchesFrontBlue;


    public double distanceToDriveOffBalanceBoardBackBlue;
    public double distanceToDriveOffBalanceBoardBackRed;
    public double distanceToCryptoBoxInchesBackRed;
    public double distanceToCryptoBoxInchesBackBlue;



    public double delayStart;
    public double speed;

    public int version;


}
