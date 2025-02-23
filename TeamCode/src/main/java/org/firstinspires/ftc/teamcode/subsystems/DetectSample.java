package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;
import com.shprobotics.pestocore.vision.ComputerVision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public class DetectSample extends OpenCvPipeline {
    ArrayList<double[]> frameList;

    // todo: tune values in FTC Dashboard

    public static Scalar lowHSV = ColorState.YELLOW.low;
    public static Scalar highHSV = ColorState.YELLOW.high;
//    public static double trapizoidWidth = 700;
    public static double blur = ColorState.YELLOW.blur;
    public static int lowThresh = ColorState.YELLOW.thresh;
    public static boolean inverted = ColorState.YELLOW.inverted;


    public enum ColorState{
        YELLOW(
                new Scalar(14, 50, 150),
                new Scalar(35, 255, 255),
                1, 1, false
        ),

        RED(
                new Scalar(14, 0, 0),
                new Scalar(255, 255, 255),
                30, 200, true
        ),

        BLUE(
                new Scalar(110, 60, 25),
                new Scalar(125, 255, 255),
                10, 100, false
        );

        ColorState(Scalar low, Scalar high, double blur, int thresh, boolean inverted) {
            this.low = low;
            this.high = high;
            this.blur = blur;
            this.thresh = thresh;
            this.inverted = inverted;
        }

        public final Scalar low, high;
        public final double blur;
        public final int thresh;
        public final boolean inverted;
    }

    ArrayList<Pose2D> positions = new ArrayList<>();
    ArrayList<Double> areas = new ArrayList<>();
    Telemetry telemetry;
    ColorState colorState = ColorState.YELLOW;

    public DetectSample(HardwareMap hardwareMap, Telemetry telemetry) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.setPipeline(this);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        
        this.telemetry = telemetry;
        this.frameList = new ArrayList<>();
    }

    public boolean cycleColorsRed(){
        if(colorState == ColorState.YELLOW)
            colorState = ColorState.RED;
        else
            colorState = ColorState.YELLOW;
        updateColorValues();
        return colorState == ColorState.YELLOW;

    }

    public boolean cycleColorsBlue(){
        if(colorState == ColorState.YELLOW)
            colorState = ColorState.BLUE;
        else
            colorState = ColorState.YELLOW;
        updateColorValues();
        return colorState == ColorState.YELLOW;
    }

    private void updateColorValues(){
        lowHSV = colorState.low;
        highHSV = colorState.high;
        blur = colorState.blur;
        lowThresh = colorState.thresh;
        inverted = colorState.inverted;
    }

    @Override
    public Mat processFrame(Mat input){
        ArrayList<Pose2D> positions = new ArrayList<>();
        ArrayList<Double> areas = new ArrayList<>();

        Mat transformMatrix = new Mat(3, 3, CvType.CV_32F);
        transformMatrix.put(0, 0,
                 +1.83e+00, +7.37e-01, -5.30e+02,
                        +1.68e-16, +1.83e+00, +0.00e+00,
                        +0.00e+00, +1.15e-03, +1.00e+00);

        Mat perspective = new Mat();
        Imgproc.warpPerspective(input, perspective, transformMatrix, input.size());

        Mat mat = ComputerVision.convertColor(perspective, Imgproc.COLOR_RGB2HSV);
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
        Mat thresh = new Mat();
        Imgproc.threshold(blurred, thresh, lowThresh, 255, Imgproc.THRESH_BINARY);


        telemetry.addData("Sample Color State", colorState);
        
        ArrayList<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresh, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);

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
            drawRotatedRect(rotatedRect, thresh, new Scalar(255, 255, 0)); //TODO TRY TO MAKE COLORED

            Pose2D position = ComputerVision.getPose(contour);
            position.add(new Vector2D(-640, -360));
            positions.add(position);

            areas.add(area);

            contour2f.release();
            contour.release();

//            telemetry.addData("X", position.getX());
//            telemetry.addData("Y", position.getY());
//            telemetry.addData("Theta", position.getHeadingRadians());
        }


        this.positions = positions;
        this.areas = areas;

        //list of frames to reduce inconsistency, not too many so that it is still real-time, change the number from 5 if you want
        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        // RELEASE EVERYTHING
        mat.release();
        transformMatrix.release();
        perspective.release();
        input.release();


//        Imgproc.line(blurred, new Point(0,720), new Point((1280-trapizoidWidth)/2, 0), new Scalar(255, 255, 0), 3);
//        Imgproc.line(blurred, new Point(1280,720), new Point((1280+trapizoidWidth)/2, 0), new Scalar(255, 255, 0), 3);

        thresh.copyTo(input);
        thresh.release();
        scaledThresh.release();
        blurred.release();

        return input;
    }

    public ArrayList<Pose2D> getPositions(){
        return positions;
    }

    public ArrayList<Double> getAreas(){
        return areas;
    }

    static void drawRotatedRect(RotatedRect rect, Mat mat, Scalar color) {
        Point[] points = new Point[4];
        rect.points(points);

        for (int i = 0; i < 4; i++) {
            Imgproc.line(mat, points[i], points[(i + 1) % 4], color, 3);
        }
    }
}