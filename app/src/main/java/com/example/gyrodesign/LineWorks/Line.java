package com.example.gyrodesign.LineWorks;

public class Line {

    public Point getPoint1() {
        return point1;
    }

    public void setPoint1(Point point1) {
        this.point1 = point1;
    }

    public Point getPoint2() {
        return point2;
    }

    public void setPoint2(Point point2) {
        this.point2 = point2;
    }

    private Point point1;
    private Point point2;

    public Line(double x1, double y1, double x2, double y2){
        point1 = new Point(x1,y1);
        point2 = new Point(x2, y2);
    }

    public Line(Point ... points){
        if(points.length ==2){
            point1 = points[0];
            point2 = points[1];
        }
    }

    public double gradient(){
        return (point2.getY()-point1.getY())/(point2.getX()-point1.getX());
    }

    public double intercept(){
        return point1.getY() - gradient()*point1.getX();
    }

    public static double gradient(double x1, double x2, double y1, double y2){
        return (y2-y1)/(x2-x1);
    }

    public String toString(){
        return "["+point1.toString()+", "+point2.toString()+"]";
    }
}
