package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.teamcode.PestoFTCConfig;
import org.firstinspires.ftc.teamcode.shplib.commands.CommandScheduler;
import org.firstinspires.ftc.teamcode.shplib.utility.Clock;

import java.util.ArrayList;

//@Disabled
@Config
@TeleOp
public class ComputerVision extends LinearOpMode {
    public static double kp = 0.0015;
    public static double kd = 0;
    ElapsedTime clawAlignment, elapsedTime;
    RotateSubsystem rotate;
    PivotSubsystem pivot;
    ClawSubsystem claw;

    Pose2D lastDetection = new Pose2D(0,0,0);


    public void runOpMode() {
        CommandScheduler.resetInstance();
        Clock.start();
        CommandScheduler.getInstance().setTelemetry(telemetry);

        DetectSample detectSample = new DetectSample(hardwareMap, telemetry);

        MecanumController mecanumController = PestoFTCConfig.getMecanumController(hardwareMap);
        ThreeWheelOdometryTracker tracker = (ThreeWheelOdometryTracker) PestoFTCConfig.getTracker(hardwareMap);

        rotate = new RotateSubsystem(hardwareMap);
        pivot = new PivotSubsystem(hardwareMap);
        claw = new ClawSubsystem(hardwareMap);

        PIDFController transPID = new PIDFController(new PIDCoefficients(
                kp, 0, kd
        ));
        transPID.setTargetPosition(0);
        transPID.setTargetVelocity(0);

        clawAlignment = new ElapsedTime();
        elapsedTime = new ElapsedTime();

        waitForStart();
        clawAlignment.reset();
        elapsedTime.reset();

        ArrayList<Pose2D> positions;
        while (opModeIsActive() && !isStopRequested()) {
            positions = detectSample.getPositions();
            if(positions.isEmpty()) {
                mecanumController.drive(0,0, 0);
                continue;
            }
            lastDetection = selectPos(positions);
            tracker.update();

//            double y, y_sign, x, x_sign;
//            y = transPID.update(lastDetection.getY(), 200*tracker.getRobotVelocity().getY());
//            y_sign = Math.signum(y);
//            y = Math.abs(y);

//            x = transPID.update(-lastDetection.getX(), 200*tracker.getRobotVelocity().getX());
//            x_sign = Math.signum(x);
//            x = Math.abs(x);

            rotate.turn(lastDetection.getHeadingRadians());
            rotate.processState();
            if(!rotate.aligned) {
                claw.setColor(ClawSubsystem.ColorState.RED);
                clawAlignment.reset();
                pivot.setState(PivotSubsystem.State.PREPARE_INTAKE_AUTO);
                claw.open();
                CommandScheduler.updateCommands();
            }
            else if(clawAlignment.seconds() > 1){
                claw.setColor(ClawSubsystem.ColorState.GREEN);
                CommandScheduler.updateCommands();
            }

//            else if(clawAlignment.seconds() > 2){
//                pivot.setState(PivotSubsystem.State.INTAKE);
//                updateCommands(0.25);
//                claw.close();
//                updateCommands(0.25);
//            }


//            mecanumController.drive(y, x, 0);

            for (Pose2D position: positions){
                telemetry.addData("X", position.getX());
                telemetry.addData("Y", position.getY());
                telemetry.addData("Theta", position.getHeadingRadians());
            }
            telemetry.addData("System nano ", System.nanoTime());
            telemetry.addData("Alignment Timer", clawAlignment.seconds());
            telemetry.addData("isCentered", sampleCentered());
            telemetry.update();
        }
    }

    public void updateCommands(double sec){
        elapsedTime.reset();
        while (elapsedTime.seconds() < sec) {
            CommandScheduler.updateCommands();
            if (isStopRequested()) return;
        }
    }


    public Pose2D selectPos(ArrayList<Pose2D> positions){
        double shortest = Double.POSITIVE_INFINITY;
        Pose2D result = new Pose2D(0,0,0);
        for(Pose2D position : positions) {
            double dist = Vector2D.dist(position.asVector(),lastDetection.asVector());
            if (dist < shortest){
                shortest = dist;
                result = position;
            }
        }
        return result;
    }

    public boolean sampleCentered(){
        double rotation = lastDetection.getHeadingRadians();

        telemetry.addData("X", lastDetection.getX());
        telemetry.addData("Y", lastDetection.getY());
        telemetry.addData("rotation", rotation);

        return abs(lastDetection.getX() - (-89)) < (107) && abs(lastDetection.getY() - (123)) < (51);
    }
}