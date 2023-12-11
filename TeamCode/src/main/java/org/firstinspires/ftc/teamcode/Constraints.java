package org.firstinspires.ftc.teamcode;

public class Constraints {
    //constructor
    public Constraints(){

    }
    //methods
    public double constrain(double var, double min, double max){
        var = Math.min(Math.max(var, min), max);
        return var;
    }
    public int constrain(int Var, int Min, int Max){
        Var = Math.min(Math.max(Var, Min), Max);
        return Var;
    }
}