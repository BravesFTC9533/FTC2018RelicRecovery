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


    Context context;
    public Config(Context context) {
        this.context = context;

    }

    public Config(Config.Colors color, Config.Positions position, boolean park, boolean jewelKnockOff, boolean cryptoBox) {
        this.color = color;
        this.position = position;
        this.Park = park;
        this.JewelKnockOff = jewelKnockOff;
        this.CryptoBox = cryptoBox;

    }

    public void Read(){

        try {
            File f = new File(context.getFilesDir(), fileName);
            if (f.exists()) {

                InputStream is = context.openFileInput(fileName);
                String jsonTxt = convertStreamToString(is);



                JSONObject json = new JSONObject(jsonTxt);

                String c = json.getString("color");
                if(c == "RED") {
                    this.color = Colors.RED;
                } else {
                    this.color = Colors.BLUE;
                }

                String p = json.getString("position");
                if(p == "FRONT") {
                    this.position = Positions.FRONT;
                } else {
                    this.position = Positions.BACK;
                }

                this.JewelKnockOff = json.getBoolean("jewel");
                this.CryptoBox = json.getBoolean("crypto");
                this.Park = json.getBoolean("park");

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
            obj.put("color", color.toString());
            obj.put("position", position.toString());
            obj.put("park", Park);
            obj.put("jewel", JewelKnockOff);
            obj.put("crypto", CryptoBox);
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









}
