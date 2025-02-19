package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;
import com.shprobotics.pestocore.vision.ComputerVision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public class DetectSample extends OpenCvPipeline {
    ArrayList<double[]> frameList;

    // todo: tune values in FTC Dashboard
    public static double lowH = 20;
    public static double highH = 40;
    public static double lowS = 0;
    public static double highS = 255;
    public static double lowV = 150;
    public static double highV = 255;
//    public static double blur = 1; //TODO TUNE ALL OF THESE
    ArrayList<Pose2D> positions = new ArrayList<>();
    Telemetry telemetry;

    public DetectSample(Telemetry telemetry) {
        this.telemetry = telemetry;
        this.frameList = new ArrayList<>();
    }

    @Override
    public Mat processFrame(Mat input){
        ArrayList<Pose2D> positions = new ArrayList<>();

        Mat mat = ComputerVision.convertColor(input, Imgproc.COLOR_RGB2HSV);
        Mat scaledThresh = ComputerVision.filterColor(mat, new Scalar(lowH, lowS, lowV), new Scalar(highH, highS, highV));
//        Mat blurred = ComputerVision.blur(scaledThresh, new Size(blur, blur)); //TODO ENP THIS IS A CNN

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
            drawRotatedRect(rotatedRect, scaledThresh, new Scalar(255, 255, 0));

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
        input.release();
        mat.release();
        scaledThresh.copyTo(input);
        scaledThresh.release();
//        blurred.release();

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