package org.firstinspires.ftc.teamcode;



/**
 * Created by 9533 on 10/28/2017.
 */
//save & write methods
    //red & blue enums
    //front & back enums
    //booleans for park
public class Config {


    public Config() {

    }

    public Config(Config.Colors color, Config.Positions position, boolean park, boolean jewelKnockOff, boolean cryptoBox) {
        this.color = color;
        this.position = position;
        this.Park = park;
        this.JewelKnockOff = jewelKnockOff;
        this.CryptoBox = cryptoBox;

    }

    public void Read(){

    }

    public void Write(){

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
