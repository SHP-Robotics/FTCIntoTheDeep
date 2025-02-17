package org.firstinspires.ftc.teamcode.autos;

import static org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem.ColorState.PINK;
import static org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem.State.WALLPICKUPAUTO;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.FConstants;
import org.firstinspires.ftc.teamcode.constants.LConstants;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;
import org.firstinspires.ftc.teamcode.teleops.AOfficialTeleOp;


@Autonomous(name = "*** Not William's PP ***")
public class PP extends OpMode {
    VerticalSubsystem vertical;
    PivotSubsystem pivot;
    RotateSubsystem rotate;
    HorizSubsystem horizontal;
    ClawSubsystem claw;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;
    private ElapsedTime elapsedTime;


    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    private final Pose startPose = new Pose(7, 64, Math.toRadians(270));
    private final Pose scorePose = new Pose(35, 72, Math.toRadians(270));
    private final Pose pickup1Pose = new Pose(61.5, 23.8, Math.toRadians(270));
    private final Pose push1Pose = new Pose(7, 24, Math.toRadians(270));
    private final Pose pickup2Pose = new Pose(7, 16, Math.toRadians(270));
    private final Pose pickup3Pose = new Pose(7, 9, Math.toRadians(270));
    private final Pose subPose = new Pose(42, 67, Math.toRadians(270));
    private final Pose pickupPose = new Pose(7, 32, Math.toRadians(270));

    private static Path scorePreload, pickup1, pickupToPush1, pickup2, pickup3, sub, grab;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));

        pickup1 = new Path(new BezierCurve(
                        new Point(scorePose),
                        new Point(new Pose(10, 8)),
                        new Point(new Pose(75, 58)),
                        new Point(pickup1Pose)));

        pickupToPush1 = new Path(new BezierLine(new Point(pickup1Pose), new Point(push1Pose)));

        pickup2 = new Path(new BezierCurve(
                new Point(push1Pose),
                new Point(new Pose(63, 40.5)),
                new Point(new Pose(78, 16)),
                new Point(new Pose(62, 6)),
                new Point(pickup2Pose)
        ));

        pickup3 = new Path(new BezierCurve(
                new Point(pickup2Pose),
                new Point(new Pose(76.5, 32.5)),
                new Point(new Pose(77.5, 0.4)),
                new Point(pickup3Pose)
        ));

        sub = new Path(new BezierLine(new Point(pickup3Pose), new Point(subPose)));

        grab = new Path(new BezierLine(new Point(subPose), new Point(pickupPose)));
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.update();
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload);
                    pathState += 1;
                }
                break;
            case 1:
                follower.update();
                if (!follower.isBusy()) {
                    follower.followPath(pickup1);
                    pathState += 1;
                }
                break;
            case 2:
                follower.update();
                if (!follower.isBusy()) {
                    follower.followPath(pickupToPush1);
                    pathState += 1;
                }
                break;
            case 3:
                follower.update();
                if (!follower.isBusy()) {
                    follower.followPath(pickup2);
                    pathState += 1;
                }
                break;
            case 4:
                follower.update();
                if (!follower.isBusy()) {
                    follower.followPath(pickup3);
                    pathState += 1;
                }
                break;
            case 5:
                follower.update();
                if (!follower.isBusy()) {
                    follower.followPath(sub);
                    pathState = 6;
                }
                break;
            case 6:
                follower.update();
                if (!follower.isBusy()) {
                    follower.followPath(grab);
                    pathState = 5;
                }
                break;
            default:
                follower.update();
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        try {
            CommandScheduler.getInstance().run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }


        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

        Clock.start();
        CommandScheduler.getInstance().setTelemetry(telemetry);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    @Override
    public void stop() {
    }


    public void updateCommands(){
        try {
            CommandScheduler.getInstance().run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public void updateCommands(double sec){
        elapsedTime.reset();
        while (elapsedTime.seconds() < sec) {
            autonomousPathUpdate();
            updateCommands();
        }
    }
    /** Prepares the intake for wall, opens claw, and closes */
    public void wallIntake(){
        //prep intake
        pivot.setState(PivotSubsystem.State.AUTOINTAKE);
        horizontal.setState(HorizSubsystem.State.DRIVING);
        updateCommands(0.25);
        rotate.setState(RotateSubsystem.State.PICKUP);
        claw.open();
        updateCommands(0.55);

        horizontal.setState(WALLPICKUPAUTO);
        updateCommands(0.05);

        claw.close();
        updateCommands(0.15);
    }

    /** Prepares passive deposit */
    public void prepArm(){
        horizontal.setState(HorizSubsystem.State.PASSIVE);
        vertical.setState(VerticalSubsystem.State.PASSIVE);
        updateCommands(0.5);

        pivot.setState(PivotSubsystem.State.PASSIVE);
        rotate.setState(RotateSubsystem.State.DROPOFF);
        updateCommands();
    }

    /** Deposits, and lowers arm */
    public void lowerArm(){
        claw.open();
        horizontal.setState(HorizSubsystem.State.DRIVING);
        pivot.setState(PivotSubsystem.State.FINISHPASSIVE);
        updateCommands(0.5);

        vertical.setState(VerticalSubsystem.State.BOTTOM);
        updateCommands(0.5);

        claw.close();
        pivot.setState(PivotSubsystem.State.DRIVING);
        updateCommands();

        pivot.setState(PivotSubsystem.State.PREPAREPICKUP);
        horizontal.setState(HorizSubsystem.State.INTAKEWALL);
        updateCommands(0.5);
        rotate.setState(RotateSubsystem.State.PICKUP);
        pivot.setState(PivotSubsystem.State.PICKUP);
        claw.open();

    }
}

