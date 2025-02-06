//package org.firstinspires.ftc.teamcode.subsystems;
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
//
//public class Vision extends Subsystem {
//    private final Limelight3A camera;
//    private boolean isDataOld = false;
//    private SampleColor detectionColor = SampleColor.BLUE;
//    private LLResult result;
//
//    private static final double CAMERA_HEIGHT = 57.0;
//    private static final double CAMERA_ANGLE = 0.0;
//    private static final double TARGET_HEIGHT = 19.05;
//
//    Telemetry telemetry;
//
//    public Vision(final HardwareMap hardwareMap) {
//        camera = hardwareMap.get(Limelight3A.class, "limelight");
//    }
//
//    public void initializeCamera() {
//        camera.setPollRateHz(100);
//        camera.start();
//    }
//
//    public enum SampleColor {
//        RED(0.0),
//        BLUE(1.0),
//        YELLOW(2.0);
//
//        private final double colorVal;
//
//        SampleColor(double colorVal) {
//            this.colorVal = colorVal;
//        }
//    }
//
//    public double getTx(double defaultValue) {
//        if (result == null) {
//            return defaultValue;
//        }
//        return result.getTx();
//    }
//
//    public double getTy(double defaultValue) {
//        if (result == null) {
//            return defaultValue;
//        }
//        return result.getTy();
//    }
//
//    public boolean isTargetVisible() {
//        if (result == null) {
//            return false;
//        }
//        return result.getTa() > 2;
//    }
//
//    public double getDistance() {
//        double ty = getTy(0.0);
//        double angleToGoalDegrees = CAMERA_ANGLE + ty;
//        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
//        double distanceMM = (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angleToGoalRadians);
//        return Math.abs(distanceMM);
//    }
//
//
//    public void periodic() {
//        camera.updatePythonInputs(
//                new double[] {detectionColor.colorVal, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
//        result = camera.getLatestResult();
//
//        if (result != null) {
//            long staleness = result.getStaleness();
//            // Less than 100 milliseconds old
//            isDataOld = staleness >= 100;
//
//            telemetry.addData("Tx", result.getTx());
//            telemetry.addData("Ty", result.getTy());
//            telemetry.addData("Ta", result.getTa());
//            telemetry.update();
//        }
//    }
//}