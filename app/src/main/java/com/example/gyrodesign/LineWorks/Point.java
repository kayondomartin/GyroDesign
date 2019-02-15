package com.example.gyrodesign.LineWorks;

public class Point {

    private double x;
    private double y;

    public Point(double x, double y){
        this.x = x;
        this.y = y;
    }

    public Point(Point point) throws NullPointerException{
        this.x = point.x;
        this.y = point.y;
    }

    public boolean equals(Point point){
        return ((this.x == point.x) && (this.y == point.y));
    }

    public Point(){
       this.x = this.y = 0;
    }

    public void setX(double x){
        this.x = x;
    }

    public void setY(double y){
        this.y = y;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public String toString(){
        return "("+x+","+y+")";
    }
}
