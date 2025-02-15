package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HorizSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RotateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VerticalSubsystem;

@Config
@Autonomous(name = "0 + 4 SAMPLE")
public class FourSample extends LinearOpMode {
    private MecanumController mecanumController;
    private DeterministicTracker tracker;
    VerticalSubsystem vertical;
    PivotSubsystem pivot;
    RotateSubsystem rotate;
    HorizSubsystem horizontal;
    ClawSubsystem claw;
    PathContainer depositBlock1, getBlock2, depositBlock2, getBlock3, depositBlock3, getBlock4, depositBlock4, park;

    PathFollower pathFollower;

    private ElapsedTime elapsedTime;

    public PathFollower generatePathFollower(PathContainer pathContainer, double deceleration, double speed) {
        return new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)
                .setEndpointPID(new PID(0.03, 0, 0.005))
                .setHeadingPID(new PID(3.0, 0, 0.02))
                .setDeceleration(2.0)
                .setSpeed(speed)
//                .setDecelerationFunction(PathFollower.SQUID_DECELERATION)
                //^^^ combats static friction
                // takes the square root of PID. PID controls drive speed
                // call this "SQUID"
                //.setCheckFinishedFunction()
                .setEndTolerance(0.4, Math.toRadians(0))
                .setEndVelocityTolerance(4)
                .setTimeAfterDeceleration(deceleration)
                .build();
    }

    @Override
    public void runOpMode() {
        CommandScheduler.resetInstance();
        PestoFTCConfig.configure();

        depositBlock1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.03)
                .addCurve(new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, 0),
                                        new Vector2D(-9, -28)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0, Math.toRadians(45) //90
                        })
                )
                .build();

        getBlock2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-9, -29),
                                        new Vector2D(-12, -19.5), //-19.5?
                                        new Vector2D(-12, -19.5),
                                        new Vector2D(-12, -19.5),
                                        new Vector2D(-12, -19.5)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(45),
                                Math.toRadians(90),
                                Math.toRadians(90),
                                Math.toRadians(90),
                                Math.toRadians(90),
                                Math.toRadians(90),
                                Math.toRadians(90) //150
                        })
                )

                .build();

        depositBlock2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-12, -19.5),
                                        new Vector2D(-8, -29)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(90),
                                Math.toRadians(45),
                                Math.toRadians(45),
                                Math.toRadians(45)
                        })
                )

                .build();

        getBlock3 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-7, -29),
                                        new Vector2D(-12, -30.25),
                                        new Vector2D(-12, -30.25),
                                        new Vector2D(-12, -30.25)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(45),
                                Math.toRadians(90),
                                Math.toRadians(90),
                                Math.toRadians(90),
                                Math.toRadians(90),
                                Math.toRadians(90)
                        })
                )

                .build();

        depositBlock3 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-12, -30.25),
                                        new Vector2D(-8, -29)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(90),
                                Math.toRadians(45),
                                Math.toRadians(45),
                                Math.toRadians(45),
                                Math.toRadians(45),
                                Math.toRadians(45)
                        })
                )

                .build();

        getBlock4 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-7, -29),
                                        new Vector2D(-15, -27.5),
                                        new Vector2D(-15, -27.5),
                                        new Vector2D(-15, -27.5),
                                        new Vector2D(-15, -27.5)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(45),
                                Math.toRadians(125),
                                Math.toRadians(125),
                                Math.toRadians(125),
                                Math.toRadians(125),
                                Math.toRadians(125)
                        })
                )

                .build();

        depositBlock4 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-15, -29),
                                        new Vector2D(-8, -27.5)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(125),
                                Math.toRadians(45),
                                Math.toRadians(45),
                                Math.toRadians(45),
                                Math.toRadians(45),
                                Math.toRadians(45)
                        })
                )

                .build();

        park = new PathContainer.PathContainerBuilder()
                .setIncrement(0.02)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-8, -27.5),
                                        new Vector2D(-60, -25),
                                        new Vector2D(-60, -25),
                                        new Vector2D(-60, -25)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(45),
                                Math.toRadians(0),
                                Math.toRadians(0),
                                Math.toRadians(0),
                                Math.toRadians(0),
                                Math.toRadians(0)
                        })
                )
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-60, -25),
                                        new Vector2D(-60, 10)
                                }
                        ),
                        new ParametricHeading(new double[]{
                                Math.toRadians(0),
                                Math.toRadians(0)
                        })
                )

                .build();

        Clock.start();
        CommandScheduler.getInstance().setTelemetry(telemetry);

        elapsedTime = new ElapsedTime();
        elapsedTime.reset();

        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        vertical = new VerticalSubsystem(hardwareMap);
        pivot = new PivotSubsystem(hardwareMap);
        rotate = new RotateSubsystem(hardwareMap);
        horizontal = new HorizSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);

        claw.close();
        pivot.processState(PivotSubsystem.State.DRIVING);

        //TODO THIS IS THE START

        waitForStart();

        //Prepare to deposit preload
        raiseArm();

        //Deposit Block 1
        followPath(depositBlock1, 0.5, 0.6); //1 to 0.5
        lowerArm();

        //Get Block 2
        prepBlock2Intake();
        followPath(getBlock2, 1, 0.6);
        startIntake();
        finishIntake();

        //Deposit Block 2
        raiseArm();
//        updateCommands(1);
        followPath(depositBlock2, 0.5, 0.6); //Deceleration 1.5 to 1
        lowerArm();

        //Get Block 3
        prepIntake();
        followPath(getBlock3, 1, 0.6);
        startIntake();
        finishIntake();

        //Deposit Block 3
        raiseArm();
        followPath(depositBlock3, 0.5, 0.6);
        lowerArm();

        //Get Block 4
        prepIntake();
        rotateIntake();
        followPath(getBlock4, 1, 0.6); //Deceleration 1.5 to 1
        startIntake();
        finishIntake();

        //Deposit Block 4
        raiseArm();
        followPath(depositBlock4, 0.5, 0.6);
        lowerArm();

        //just for good measure
        vertical.setState(VerticalSubsystem.State.BOTTOM);
        updateCommands();

        parkArm();
        followPath(park, 1, 0.6);
        updateCommands();

    }

    public void loopOpMode() {
        try {
            CommandScheduler.getInstance().run();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        tracker.update();
        pathFollower.update();

        telemetry.addData("Loop Times", elapsedTime.milliseconds());
        telemetry.addData("x", tracker.getCurrentPosition().getX());
        telemetry.addData("y", tracker.getCurrentPosition().getY());
        telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
        elapsedTime.reset();
        telemetry.update();
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
            tracker.update();
            updateCommands();
            if (isStopRequested()) return;
        }
    }
    public void followPath(PathContainer path, double deceleration, double speed){
        if (isStopRequested()) return;

        pathFollower = generatePathFollower(path, deceleration, speed);

        while (opModeIsActive() && !isStopRequested() && !pathFollower.isCompleted()) {
            loopOpMode();
        }
    }

    /** Prepares the intake sample */
    public void prepIntake(){
        //prep intake
        horizontal.setState(HorizSubsystem.State.PREPAUTOINTAKE);
        pivot.setState(PivotSubsystem.State.PREPAREINTAKE);
        rotate.setState(RotateSubsystem.State.INTAKE);
        updateCommands(0.25);
        claw.open();
        horizontal.setState(HorizSubsystem.State.INTAKINGEXTENDED);
        updateCommands();
    }
    public void prepBlock2Intake(){
        //prep intake
        horizontal.setState(HorizSubsystem.State.PREPAUTOINTAKE);
        pivot.setState(PivotSubsystem.State.PREPAREINTAKEHIGHER);
        rotate.setState(RotateSubsystem.State.INTAKE);
        updateCommands(0.25);
        claw.open();
        horizontal.setState(HorizSubsystem.State.INTAKINGEXTENDED);
        updateCommands();
    }
    /** Prepares the intake sample */
    public void rotateIntake(){
        //prep intake
        rotate.setState(RotateSubsystem.State.SAMPLE);
        updateCommands();
    }
    /** Grabs sample */
    public void startIntake(){
        pivot.setState(PivotSubsystem.State.INTAKE);
        updateCommands(0.25);
        claw.close();
        updateCommands(0.25);

        //reattempt if fail
        if(!claw.isBlockInClaw()){
            claw.setColor(ClawSubsystem.ColorState.RED);

            pivot.setState(PivotSubsystem.State.PREPAREINTAKE);
            claw.open();
            updateCommands(0.25);

            pivot.setState(PivotSubsystem.State.INTAKE);
            updateCommands(0.25);
            claw.close();
            updateCommands(0.25);

            if(claw.isBlockInClaw()) {
                claw.setColor(ClawSubsystem.ColorState.GREEN);
                updateCommands();
            }
        }

    }
    /** Returns to driving mode */
    public void finishIntake(){
        rotate.setState(RotateSubsystem.State.NEUTRAL);
        pivot.setState(PivotSubsystem.State.DRIVING);
        horizontal.setState(HorizSubsystem.State.DRIVING);
        updateCommands(0.25); //removed wait
        claw.setColor(ClawSubsystem.ColorState.OFF);
        updateCommands();
    }

    /** Raises the Vertical */
    public void raiseArm(){
        claw.setColor(ClawSubsystem.ColorState.OFF);
        vertical.setDepositState(VerticalSubsystem.State.HIGHBUCKET);
        vertical.setState(VerticalSubsystem.State.DEPOSITING);
        updateCommands();

        horizontal.setState(HorizSubsystem.State.DRIVING);
        updateCommands(1);
        pivot.setState(PivotSubsystem.State.OUTTAKEBUCKET);
        updateCommands(0.5);
        rotate.setState(RotateSubsystem.State.DROPOFFBUCKET);
        updateCommands(0.25); // 0.5 -> 0.25
    }

    /** Deposits, and lowers arm */
    public void lowerArm(){
//        updateCommands(0.5);

        claw.open();
//        updateCommands(0.5);

        rotate.setState(RotateSubsystem.State.DROPOFF);
        updateCommands(0.25);
        pivot.setState(PivotSubsystem.State.DRIVING);
        claw.close();
        vertical.setState(VerticalSubsystem.State.BOTTOM);
        updateCommands(0.5); //removed wait
    }

    public void parkArm(){
        //wrist: 0
        //elbow: .355
        //mgn: 0.725
        pivot.setState(PivotSubsystem.State.PARK);
        horizontal.setState(HorizSubsystem.State.PARK);
        updateCommands();
    }
}