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
    public PathFollower generatePathFollower(PathContainer pathContainer, double deceleration, double speed) {
        return new PathFollower.PathFollowerBuilder(mecanumController, tracker, pathContainer)

                .setHeadingPID(new PID(0, 0, 0))
                .setEndpointPID(new PID(0,0,0))
                .setDeceleration(PestoFTCConfig.DECELERATION)

                .setSpeed(speed)
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

        PathContainer Intake1 = new PathContainer.PathContainerBuilder()
                .setIncrement(0.01)
                .addCurve(

                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(0, 0),
                                        new Vector2D(0,48),
//                                        new Vector2D(100, 40),

                                }
                        ),     new ParametricHeading(new double[]{
                                0,0
                        })

                )
                .build();

//        PathContainer Outtake1 = new PathContainer.PathContainerBuilder()
//                .setIncrement(0.01)
//                .addCurve(
//
//                        new BezierCurve(
//                                new Vector2D[]{
//                                        new Vector2D(0, 0),
//                                        new Vector2D(30, 0),
//                                        new Vector2D(30, 38),
//                                }
//
//                        )
//
//                )
//                .build();

        PathContainer Park = new PathContainer.PathContainerBuilder()
                .setIncrement(0.01)
                .addCurve(
                        new BezierCurve(
                                new Vector2D[]{
                                        new Vector2D(20, 0),
                                }
                        ),
                        new ParametricHeading(new double[]{
                                0,0
                        })
                )
                .build();
        waitForStart();
        clawSubsystem.setClose();
        clawSubsystem.update();
        tracker.reset();
        wormGearSubsystem.setToZero(touchSensor, telemetry);
        followPath(Intake1, 2, 0.4);
        cycleAndUpdateHang();
        waitSec(1.0);
        cycleAndUpdateHang();
        waitSec(0.5);
        clawSubsystem.setOpen();
        clawSubsystem.update();
    }

    public void loopOpMode () {
        telemetry.addData("position", tracker.getCurrentPosition());
        telemetry.addData("velocity", tracker.getRobotVelocity());
        telemetry.addData("Path Progress", pathFollower.isCompleted() ? "Completed" : "In Progress");
        tracker.update();
        pathFollower.update();
        telemetry.update();
        wormGearSubsystem.updateTelemetry(telemetry);
    }

    public void followPath (PathContainer path,double deceleration, double speed){
        if (isStopRequested()) return;
        pathFollower = generatePathFollower(path, deceleration, speed);
        while (opModeIsActive() && !isStopRequested() && !pathFollower.isCompleted()) {
            loopOpMode();
        }
    }

    public void cycle(){
        wormGearSubsystem.cycle();
        viperSlideSubsystem.cycle();
    }
    public void cycleHang(){
        wormGearSubsystem.cycleHanging();
        viperSlideSubsystem.cycleHanging();
    }
    public void updateHang(){
        wormGearSubsystem.updateHanging();
        viperSlideSubsystem.updateHanging();
    }
    public void cycleAndUpdateHang(){
        cycleHang();
        elapsedTime.reset();
        while (elapsedTime.seconds()<1){
            updateHang();
        }
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



