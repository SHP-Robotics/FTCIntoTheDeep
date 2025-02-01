package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.opencv.core.Mat;

@Autonomous
public class DontPressSquareAutonomous extends LinearOpMode {
    MecanumController mecanumController;
    DeterministicTracker tracker;
    PathFollower pathFollower;
    ViperSlideSubsystem viperSlideSubsystem;
    WormGearSubsystem wormGearSubsystem;
    ClawSubsystem clawSubsystem;
    ElapsedTime elapsedTime;
    TouchSensor touchSensor;
    PathContainer Outtake1;
    PathContainer Intake1;
    PathContainer Outtake2;
    PathContainer Outtake3;


    Runnable Step1;
    Runnable Step2;
    Runnable Step3;
    Runnable Step4;



    int step=0;


    public PathFollower generatePathFollower(PathContainer pathContainer, double deceleration, double speed, Runnable action, double killTime) {
        return new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)
                .addFinalAction(action)
                .setHeadingPID(new PID(0.3, 0, 0))
                .setEndpointPID(new PID(0.0003,0,0))
                .setDeceleration(PestoFTCConfig.DECELERATION)
                .setEndTolerance(0.5, Math.toRadians(0.5))
                .setEndVelocityTolerance(2)
                .setSpeed(speed)
                .setKillTime(killTime)
                .build();


    }
    @Override
    public void runOpMode() throws InterruptedException {
        elapsedTime=new ElapsedTime();
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");

        mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        tracker = PestoFTCConfig.getTracker(hardwareMap);
        tracker.reset();
        wormGearSubsystem = new WormGearSubsystem(hardwareMap);

        viperSlideSubsystem = new ViperSlideSubsystem(hardwareMap);
        clawSubsystem = new ClawSubsystem(hardwareMap);

        Step1 = () -> {


            waitSec(0.5);
            cycleHang();
            updateHang();
            waitSec(0.2);
            clawSubsystem.setOpen();
            clawSubsystem.update();
            waitSec(1);
            resetCycles();
//        step++;
            waitSec(0.1);
            followPath(Intake1, 0.5, 1, Step2,2);
//        step++;

        };
        Step2 = () -> {
//            step=2;
//            waitSec(1);
//            cycle();
//            update();
//            clawSubsystem.setClose();
//            clawSubsystem.update();
//            waitSec(0.1);
//            cycle();
//            update();
            followPath(Outtake2,0.5,1,Step3,2);

        };
        Step3 = () -> {
//            cycle();
//            update();
//            clawSubsystem.setOpen();
//            clawSubsystem.update();
            waitSec(0.5);
            followPath(Outtake3,0.5,1,Step4,2);

        };

        Outtake1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.01)
                .addCurve(

                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, 0),
                                        new Vector2D(-10, 0),
                                        new Vector2D(-10,25),

                                }
                        )
                        ,     new ParametricHeading(new double[]{
                                0,Math.toRadians(-0)
                        }
                        )


                )
                .build();
        Intake1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.01)
                .addCurve(

                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(-10, 25),
                                        new Vector2D(0,15),
                                        new Vector2D(15,18),
                                        new Vector2D(15,50),




                                }
                        )
                        ,     new ParametricHeading(new double[]{
                                0,Math.toRadians(-0)
                        }
                        )


                )
                .build();
        Outtake2 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.01)
                .addCurve(

                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(15, 50),
                                        new Vector2D(37,42),
                                        new Vector2D(37,10),



                                }
                        )
                        ,     new ParametricHeading(new double[]{
                                0,Math.toRadians(0)
                        }
                        )


                )
                .build();
        Outtake3 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.01)
                .addCurve(

                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(37, 10),
                                        new Vector2D(0,0),
                                }
                        )
                        ,     new ParametricHeading(new double[]{
                                0,Math.toRadians(0)
                        }
                        )


                )
                .build();

        waitForStart();
        wormGearSubsystem.setToZero(touchSensor, telemetry);
        clawSubsystem.setClose();
        clawSubsystem.update();

        tracker.reset();
        resetCycles();

        cycleHang();
        updateHang();
        followPath(Outtake1, 0.5, 0.5, Step1, 2);


    }

    public void loopOpMode () {
        telemetry.addData("position", tracker.getCurrentPosition());
        telemetry.addData("velocity", tracker.getRobotVelocity());
        telemetry.addData("Path Progress", pathFollower.isCompleted() ? "Completed" : "In Progress");
        telemetry.addData("STEP", step);
        tracker.update();
        pathFollower.update();
        telemetry.update();


        wormGearSubsystem.updateTelemetry(telemetry);
        if(step<2) {
            updateHang();
        }else if (step==2){
            update();
        }

    }
    public void followPath (PathContainer path,double deceleration, double speed, Runnable action, double killTime){
        if (isStopRequested()) return;
        pathFollower = generatePathFollower(path, deceleration, speed,action, killTime);
        while (opModeIsActive() && !isStopRequested() && !pathFollower.isCompleted()) {
            loopOpMode();
        }
    }

    public void cycle(){
        wormGearSubsystem.cycle();
        viperSlideSubsystem.cycle();
    }
    public void resetCycles(){
        wormGearSubsystem.resetCycles();
        viperSlideSubsystem.resetCycles();
    }
    public void cycleHang(){
        wormGearSubsystem.cycleHanging();
        viperSlideSubsystem.cycleHanging();
        wormGearSubsystem.updateHanging();
        viperSlideSubsystem.updateHanging();
    }
    public void updateHang(){
        wormGearSubsystem.updateHanging();
        viperSlideSubsystem.updateHanging();
    }
    public void waitSec(double second){
        elapsedTime.reset();
        while (elapsedTime.seconds()<second){
        }
    }
    public void update(){
        wormGearSubsystem.update();
        viperSlideSubsystem.update();
    }

}



