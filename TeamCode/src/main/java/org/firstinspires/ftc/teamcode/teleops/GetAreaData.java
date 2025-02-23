package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.geometries.Pose2D;

import org.apache.commons.math3.stat.descriptive.SummaryStatistics;
import org.firstinspires.ftc.teamcode.subsystems.DetectSample;

import java.util.ArrayList;

@TeleOp
public class GetAreaData extends LinearOpMode {
    @Override
    public void runOpMode() {
        DetectSample detectSample = new DetectSample(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.b) {
                ArrayList<Double> areas = detectSample.getAreas();
                telemetry.addData("num positions", areas.size());

                if (areas.size() == 1)
                    telemetry.addData("area", areas.get(0));
            }

            telemetry.update();
        }
    }
}
