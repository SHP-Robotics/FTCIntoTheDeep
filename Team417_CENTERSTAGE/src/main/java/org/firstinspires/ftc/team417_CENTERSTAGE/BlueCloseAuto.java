package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
<<<<<<< HEAD
@Autonomous (name="RIGHT SIDE")
public class BlueCloseAuto extends BaseAutonomous {
=======
@Autonomous (name="Blue Close ")

public class LowJunctionParkRightSide extends BaseAutonomous {
>>>>>>> df5fd88d01499846b8bdecb7f23d254a034f6d28

    @Override
    public void runOpMode() {

        initializeAuto();

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        TrajectorySequence untitled1 = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -60.00, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-36.00, -37.00), Math.toRadians(90.00))
                .splineTo(new Vector2d(-36.00, -60.00), Math.toRadians(105.00))
                .splineTo(new Vector2d(-58.56, -35.09), Math.toRadians(90.00))
                .splineTo(new Vector2d(-48.00, -12.00), Math.toRadians(360.00))
                .splineTo(new Vector2d(60.00, -12.00), Math.toRadians(360.00))
                .build();
    }
}