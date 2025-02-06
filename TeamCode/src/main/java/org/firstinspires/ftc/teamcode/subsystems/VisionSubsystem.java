//package org.firstinspires.ftc.teamcode.subsystems;
//
//
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.teamcode.shplib.commands.Subsystem;
//
//import java.util.List;
//
//public class VisionSubsystem extends Subsystem {
//
//    public Limelight3A limelight;
//
//    public VisionSubsystem(HardwareMap hardwareMap){
//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(2); //THIS IS BLUE
//    }
//
//
//
//    @Override
//    public void periodic(Telemetry telemetry) {
//        telemetry.setMsTransmissionInterval(11);
//
//        LLResult result = limelight.getLatestResult();
//        if (result != null) {
//            if (result.isValid()) {
//
//                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
//
//                List<List<Double>> coordinates = colorResults.get(0).getTargetCorners();
//
//                for(List<Double> coord : coordinates) {
//                    telemetry.addData("Corner X: ", coord.get(0));
//                    telemetry.addData("Corner Y: ", coord.get(1));
//                }
//
//            }
//        }
//
//    }
//
//
//}
