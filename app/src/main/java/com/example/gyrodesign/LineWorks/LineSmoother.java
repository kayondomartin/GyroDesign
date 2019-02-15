package com.example.gyrodesign.LineWorks;

import android.util.Log;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.concurrent.ConcurrentHashMap;

public class LineSmoother {

    public static double mean(List<Point> points){
        double mean = 0;

        for(Point point: points){
            mean += point.getY();
        }
        mean /= points.size();

        return mean;
    }

    public static class NonEqualException extends Exception{
        private int num1;
        private int num2;

        public NonEqualException(int x, int y){
            num1 = x; num2 = y;
        }

        public String toString(){
            return "NonEqualException: Num1 = "+num1+", Num2 = "+num2+"\n";
        }
    }
    public static double corrCoeff(List<Point> x, List<Point> y) throws NonEqualException{

            int sizeX = x.size();
            int sizeY = y.size();

            if (sizeX != sizeY) {
                throw new NonEqualException(sizeX,sizeY);
            }
            double mx = mean(x);
            double my = mean(y);

            double Exy = 0, Ex2 = 0, Ey2 = 0;


            Log.e(LineSmoother.class.getSimpleName(), "CorrSize: " + sizeX);

            for (int i = 0; i < sizeX; i++) {
                Exy += (x.get(i).getY() - mx) * (y.get(i).getY() - my);
                Ex2 += Math.pow((x.get(i).getY() - mx), 2);
                Ey2 += Math.pow((y.get(i).getY() - my), 2);
            }

            double corr = Exy / (Math.sqrt(Ex2 * Ey2));
            Log.e(LineSmoother.class.getSimpleName(), "Corr: " + corr);
            return corr;
    }

    public static List<Point> sample(List<Line> lines,double start, double step, double end){

        List<Point> points = new ArrayList<>();

        for(double i=start;i<=end;i+=step){
            for(Line line: lines ){
                if((line.getPoint1().getX()<=i && line.getPoint2().getX()>= i) || (line.getPoint1().getX()>=i && line.getPoint2().getX()<= i) ){
                    double m = line.gradient();
                    double c = line.intercept();
                    double y = m*i + c;

                    points.add(new Point(i,y));
                }
            }
        }

        return points;
    }


    public static List<Point> standardize(List<Point> points){
        double min = getMin(points);
        double max = getMax(points);
        List<Point> standardized = new ArrayList<>();

        int i;
        int length = points.size();
        for(i=0;i<length;i++){
            Point point = points.get(i);
            double standard = 2*((point.getY()-min)/(max-min)) - 1;
            standardized.add(new Point(point.getX(),standard));
        }
        return standardized;
    }


    private static double getMax(List<Point> points){
        double max = points.get(0).getY();

        for (Point point: points){
            if(point.getY() > max) max = point.getY();
        }

        return max;
    }

    private static double getMin(List<Point> points){
        double min = points.get(0).getY();

        for(Point point: points){
            if(point.getY() < min) min = point.getY();
        }

        return min;
    }

    public static List<Line> smoothLine(List<Point> points){
        if(points.size() < 5){
            List<Line> lines = new ArrayList<>();

            for(int i = 0;i<points.size()-1;i++){
                lines.add(new Line(points.get(i),points.get(i+1)));
            }

            return lines;
        }

        List<Line> smoothedLine = new ArrayList<>();
        smoothedLine.add(new Line(new Point(points.get(0)), new Point(points.get(1))));

        Point newPoint = points.get(1);
        int i;
        int size = points.size()-2;
        for(i=2;i<size;i++){
            Point lastPoint = newPoint;
            newPoint = smoothPoint(points.subList(i-2,i+3));
            smoothedLine.add(new Line(new Point(lastPoint), new Point(newPoint)));
        }

        Line lastSegment = new Line(new Point(points.get(size)), new Point(points.get(size+1)));
        smoothedLine.add(new Line(new Point(newPoint),new Point(lastSegment.getPoint1())));
        smoothedLine.add(lastSegment);

        return smoothedLine;
    }

    public static List<Line> smoothLine(ConcurrentHashMap<Integer, Point> linePoints, int size){
        if(size < 5){
            List<Line> lines = new ArrayList<>();
            for(int i=1;i<size;i++){
                lines.add(new Line(linePoints.get(i),linePoints.get(i+1)));
            }

            return lines;
        }

        List<Line> smoothedLine = new ArrayList<>();
        List<Point> points = new ArrayList<>();

        for(int i=1; i<=size;i++){
            points.add(new Point(linePoints.get(i)));
        }

        smoothedLine.add(new Line(new Point(linePoints.get(1)), new Point(linePoints.get(2))));

        Point newPoint = points.get(1);

        for(int i = 2; i< points.size()-2; i++){
            Point lastPoint = newPoint;
            newPoint = smoothPoint(points.subList(i-2,i+3));
            smoothedLine.add(new Line(lastPoint, newPoint));
        }

        Line lastSegment = new Line(new Point(linePoints.get(size-1)),new Point(linePoints.get(size)));
        smoothedLine.add(new Line(newPoint,new Point(lastSegment.getPoint1())));

        smoothedLine.add(lastSegment);

        return smoothedLine;
    }

    private static Point smoothPoint(final List<Point> points) {

        double avgX = 0;
        double avgY = 0;

        for(Point point: points){
            avgX += point.getX();
            avgY += point.getY();
        }

        avgX /= points.size();
        avgY /= points.size();

        Point newPoint = new Point(avgX,avgY);
        Point oldPoint = points.get(points.size()/2);
         double newX = (newPoint.getX() + oldPoint.getX())/2;
         double newY = (newPoint.getY() + oldPoint.getY())/2;

        return new Point(newX,newY);
    }

    private static List<Point> getPoints(ConcurrentHashMap<Integer, Line> map, Point lastPoint){

        List<Point> points = new ArrayList<>();

        for(Object o: map.entrySet()){
            Line line = (Line) o;
            points.add(line.getPoint1());
        }

       return points;
    }


    private static List<Point> getPoints(List<Line> lineSegments) {

        List<Point> points =  new ArrayList<>();
        Iterator itr = lineSegments.iterator();

        while(itr.hasNext()){
            Line segment = (Line)itr.next();
            points.add(new Point(segment.getPoint1()));
        }
        points.add(new Point(lineSegments.get(lineSegments.size()-1).getPoint2()));
        return points;
    }
}