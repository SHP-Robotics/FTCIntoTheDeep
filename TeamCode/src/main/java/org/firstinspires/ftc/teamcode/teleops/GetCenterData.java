package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.geometries.Pose2D;

import org.apache.commons.math3.stat.descriptive.SummaryStatistics;
import org.firstinspires.ftc.teamcode.subsystems.DetectSample;

import java.util.ArrayList;

@TeleOp
public class GetCenterData extends LinearOpMode {
    SummaryStatistics xData;
    SummaryStatistics yData;

    @Override
    public void runOpMode() {
        DetectSample detectSample = new DetectSample(hardwareMap, telemetry);

        xData = new SummaryStatistics();
        yData = new SummaryStatistics();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.b) {
                ArrayList<Pose2D> positions = detectSample.getPositions();
                telemetry.addData("num positions", positions.size());

                if (positions.size() == 1) {
                    Pose2D position = positions.get(0);
                    xData.addValue(position.getX());
                    yData.addValue(position.getY());
                }
            }

            telemetry.addData("x mean", xData.getMean());
            telemetry.addData("x std", xData.getStandardDeviation());

            telemetry.addData("y mean", yData.getMean());
            telemetry.addData("y std", yData.getStandardDeviation());

            telemetry.update();
        }
    }
}
