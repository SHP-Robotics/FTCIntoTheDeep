package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;
import com.shprobotics.pestocore.vision.ComputerVision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public class DetectSample extends OpenCvPipeline {
    ArrayList<double[]> frameList;

    // todo: tune values in FTC Dashboard

    public static Scalar lowYellow = new Scalar(20, 0, 150);
    public static Scalar highYellow = new Scalar(40, 255, 255);
    public static Scalar lowRed = new Scalar(10, 0, 0); //TODO TUNE BLUE AND RED
    public static Scalar highRed = new Scalar(225, 225, 225);
    public static Scalar lowBlue = new Scalar(40, 45, 25);
    public static Scalar highBlue = new Scalar(180, 255, 255);

    public static Scalar lowHSV = lowYellow;
    public static Scalar highHSV = highYellow;
    public static double trapizoidWidth = 700;
    public static boolean inverted = false;

    public enum ColorState{
        YELLOW, RED, BLUE;
    }

    public static double blur = 1;
    ArrayList<Pose2D> positions = new ArrayList<>();
    Telemetry telemetry;
    ColorState colorState = ColorState.YELLOW;

    public DetectSample(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.frameList = new ArrayList<>();
    }

    public boolean cycleColorsRed(){
        if(colorState == ColorState.YELLOW){
            colorState = ColorState.RED;
            lowHSV = lowRed;
            highHSV = highRed;
            inverted = true;
            return false;
        }
        else{
            colorState = ColorState.YELLOW;
            lowHSV = lowYellow;
            highHSV = highYellow;
            inverted = false;
            return true;
        }
    }

    public boolean cycleColorsBlue(){
        if(colorState == ColorState.YELLOW){
            colorState = ColorState.BLUE;
            lowHSV = lowBlue;
            highHSV = highBlue;
            return false;
        }
        else {
            colorState = ColorState.YELLOW;
            lowHSV = lowYellow;
            highHSV = highYellow;
            return true;
        }
    }

    @Override
    public Mat processFrame(Mat input){
        ArrayList<Pose2D> positions = new ArrayList<>();

        Mat transformMatrix = new Mat(3, 3, CvType.CV_32F);
        transformMatrix.put(0, 0,
                1.01150794e+00, 7.90240575e-03, -1.01150794e+01,
                -1.27794308e-17, 5.81546506e-01, 9.75715091e-16
                        -6.29340278e-04, 2.64534810e-05, 1.00000000e+00);

        Mat perspective = new Mat();
        Core.perspectiveTransform(input, perspective, transformMatrix);

        Mat mat = ComputerVision.convertColor(input, Imgproc.COLOR_RGB2HSV);
        Mat scaledThresh;

        if(inverted){
            Mat notScaledThresh = ComputerVision.filterColor(mat, lowHSV, highHSV);
            scaledThresh = new Mat();
            Core.bitwise_not(notScaledThresh, scaledThresh);
            notScaledThresh.release();
        }
        else
            scaledThresh = ComputerVision.filterColor(mat, lowHSV, highHSV);

        Mat blurred = ComputerVision.blur(scaledThresh, new Size(blur, blur)); //TODO ENP THIS IS A CNN
//        Mat sharp = Imgproc.threshold(blurred, sharp, 0.5,)
        telemetry.addData("Sample Color State", colorState);
        
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(scaledThresh, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

        for (MatOfPoint contour: contours) {
            Point[] points = contour.toArray();
            MatOfPoint2f contour2f = new MatOfPoint2f(points);
            RotatedRect rotatedRect = Imgproc.minAreaRect(contour2f);

            Moments moments = Imgproc.moments(contour);
            double area = moments.m00;

            if (area < 100000) //TODO TUNE
                continue; //skip if shape is too small or just noise

            // todo: delete telemetry after tuning
//            telemetry.addLine("h" + rotatedRect.size.height + " w" + rotatedRect.size.width);
//            drawRotatedRect(rotatedRect, blurred, new Scalar(255, 255, 0)); //TODO TRY TO MAKE COLORED

            Pose2D position = ComputerVision.getPose(contour);
            position.add(new Vector2D(-640, -360));
            positions.add(position);

            contour2f.release();
            contour.release();

//            telemetry.addData("X", position.getX());
//            telemetry.addData("Y", position.getY());
//            telemetry.addData("Theta", position.getHeadingRadians());
        }


        this.positions = positions;

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        // RELEASE EVERYTHING
        mat.release();

        Imgproc.line(blurred, new Point(0,720), new Point((1280-trapizoidWidth)/2, 0), new Scalar(255, 255, 0), 3);
        Imgproc.line(blurred, new Point(1280,720), new Point((1280+trapizoidWidth)/2, 0), new Scalar(255, 255, 0), 3);

        blurred.copyTo(input);

        scaledThresh.release();
        blurred.release();

        return input;
    }

    public ArrayList<Pose2D> getPositions(){
        return positions;
    }

    static void drawRotatedRect(RotatedRect rect, Mat mat, Scalar color) {
        Point[] points = new Point[4];
        rect.points(points);

        for (int i = 0; i < 4; i++) {
            Imgproc.line(mat, points[i], points[(i + 1) % 4], color, 3);
        }
    }
}