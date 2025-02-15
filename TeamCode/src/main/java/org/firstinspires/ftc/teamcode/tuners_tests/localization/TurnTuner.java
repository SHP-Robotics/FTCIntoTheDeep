package org.firstinspires.ftc.teamcode.tuners_tests.localization;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.Constants;
import com.pedropathing.util.DashboardPoseTracker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;

/**
 * This is the TurnTuner OpMode. This tracks the turning movement of the robot and displays the
 * necessary ticks to inches multiplier. This displayed multiplier is what's necessary to scale the
 * robot's current angle in ticks to the specified angle in radians. So, to use this, run the
 * tuner, then pull/push the robot to the specified angle using a protractor or lines on the ground.
 * When you're at the end of the angle, record the ticks to inches multiplier. Feel free to run
 * multiple trials and average the results. Then, input the multiplier into the turning ticks to
 * radians in your localizer of choice.
 * You can adjust the target angle on FTC Dashboard: 192/168/43/1:8080/dash
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 5/6/2024
 */

@Disabled
@Config
@Autonomous(name = "Turn Localizer Tuner", group = ".Localization")
public class TurnTuner extends OpMode {
    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;

    private Telemetry telemetryA;

    public static double ANGLE = 2 * Math.PI;

    /**
     * This initializes the PoseUpdater as well as the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        poseUpdater = new PoseUpdater(hardwareMap);

        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        telemetry.addLine("Turn your robot " + ANGLE + " radians. Your turn ticks to inches will be shown on the telemetry.");
        telemetry.update();
    }

    /**
     * This updates the robot's pose estimate, and updates the FTC Dashboard telemetry with the
     * calculated multiplier and draws the robot.
     */
    @Override
    public void loop() {
        poseUpdater.update();

        telemetry.addData("total angle", poseUpdater.getTotalHeading());
        telemetry.addLine("The multiplier will display what your turn ticks to inches should be to scale your current angle to " + ANGLE + " radians.");
        telemetry.addData("multiplier", ANGLE / (poseUpdater.getTotalHeading() / poseUpdater.getLocalizer().getTurningMultiplier()));
    }
}
