package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Date;
import java.util.List;

@Config
@Autonomous(name = "Center Stage Backdrop Blue", group = "competition")
public class CenterStageBackdropBlue extends LinearOpMode {
    private OpenCvCamera webcam;

    private static final int CAMERA_WIDTH  = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    enum State {
        PURPLE_DEPOSIT_PATH,
        PURPLE_DEPOSIT,
        PURPLE_BACKUP,

        UNTURN,
        BACKUP_FROM_SPIKES,

        YELLOW_DEPOSIT_PATH,
        YELLOW_DEPOSIT_STRAIGHT,
        YELLOW_DEPOSIT,
        BACK,
        PARK,
        STOP
    }

    ElapsedTime depositTime = new ElapsedTime();
    int resetInt = 0;

    SampleMecanumDrive drive;

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "frontWebcam"), cameraMonitorViewId);
        PropFindLeft myPipeline;
        webcam.setPipeline(myPipeline = new PropFindLeft(telemetry,packet));
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //CenterStageComputerVisionPipelines CV = new CenterStageComputerVisionPipelines(hardwareMap, telemetry);
        PropFindLeft.pos propPos = null;

        drive = new SampleMecanumDrive(hardwareMap, telemetry);
        Pose2d startPose = new Pose2d(new Vector2d(12, 58.5), Math.toRadians(270)); //Start position for roadrunner
        drive.setPoseEstimate(startPose);

        State currentState;

        Trajectory purpleDepositPathL = drive.trajectoryBuilder(startPose,false)
                .lineToSplineHeading(new Pose2d(new Vector2d(9.5, 38), Math.toRadians(305)))
                .build();

        Trajectory purpleDepositPathR = drive.trajectoryBuilder(startPose,false)
                .lineToSplineHeading(new Pose2d(new Vector2d(11, 32.5), Math.toRadians(210)))
                .build();

        Trajectory purpleDepositPathC = drive.trajectoryBuilder(startPose,false)
                .lineToSplineHeading(new Pose2d(new Vector2d(12, 36.5), Math.toRadians(270)))
                .build();

        Trajectory purpleBackUpC = drive.trajectoryBuilder(purpleDepositPathC.end(),false)
                .back(3)
                .build();

        Trajectory purpleBackUpR = drive.trajectoryBuilder(purpleDepositPathR.end(), false)
                .back(4)
                .build();

        Trajectory purpleBackUpL = drive.trajectoryBuilder(purpleDepositPathL.end(), false)
                .back(3)
                .build();

        Trajectory backUpFromSpikes = drive.trajectoryBuilder(purpleBackUpC.end(),false)
                .back(3.3)
                .build();

        Trajectory yellowDepositPathC = drive.trajectoryBuilder(backUpFromSpikes.end(),false)
                .splineToLinearHeading(new Pose2d(new Vector2d(44, 30), Math.toRadians(180)), Math.toRadians(-60))
                .build();

        Trajectory yellowDepositPathL = drive.trajectoryBuilder(backUpFromSpikes.end(),false)
                .splineToLinearHeading(new Pose2d(new Vector2d(44, 34), Math.toRadians(180)), Math.toRadians(-60))
                .build();

        Trajectory yellowDepositPathR = drive.trajectoryBuilder(backUpFromSpikes.end(),false)
                .splineToLinearHeading(new Pose2d(new Vector2d(45, 24), Math.toRadians(180)), Math.toRadians(-60))
                .build();

        Trajectory parkC = drive.trajectoryBuilder(yellowDepositPathC.end())
                .back(4)
                .build();
        Trajectory parkL = drive.trajectoryBuilder(yellowDepositPathL.end())
                .back(4)
                .build();
        Trajectory parkR = drive.trajectoryBuilder(yellowDepositPathR.end())
                .back(4)
                .build();


        Trajectory park;

        int target=0;

        drive.closeClaw();

        propPos = myPipeline.position;

        waitForStart();

        drive.closeClaw();


        long startTime = new Date().getTime();
        long time = 0;

        while (time < 50 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            propPos = myPipeline.position;
            telemetry.addData("Position", propPos);
        }

        currentState = State.PURPLE_DEPOSIT_PATH;
        if (propPos == PropFindLeft.pos.LEFT) {
            drive.followTrajectoryAsync(purpleDepositPathL);
        } else if (propPos == PropFindLeft.pos.RIGHT) {
            drive.followTrajectoryAsync(purpleDepositPathR);
        } else {
            drive.followTrajectoryAsync(purpleDepositPathC);
        }

        while (opModeIsActive() && !isStopRequested()) {
            drive.update();
            drive.backSlidesMove(target);

            telemetry.addData("current state", currentState.name());

            switch (currentState) {
                case PURPLE_DEPOSIT_PATH:
                    if (!drive.isBusy()) {
                        currentState = State.PURPLE_DEPOSIT;

                    } else {
                        drive.intakeToGround();
                    }
                    break;
                case PURPLE_DEPOSIT:
                    drive.openClaw();
                    drive.closeHook();
                    sleep(500);
                    if (propPos == PropFindLeft.pos.LEFT){
                        drive.followTrajectory(purpleBackUpL);
                    } else if (propPos== PropFindLeft.pos.RIGHT){
                        drive.followTrajectory(purpleBackUpR);
                    } else {
                        drive.followTrajectoryAsync(purpleBackUpC);
                    }
                    currentState= State.PURPLE_BACKUP;
                    break;
                case PURPLE_BACKUP:
                    if (!drive.isBusy()) {
                        drive.intakeToTransfer();
                        sleep(300);
                        if (propPos == PropFindLeft.pos.LEFT) {
                            drive.turn(Math.toRadians(-45));
                        } else if (propPos == PropFindLeft.pos.RIGHT) {
                            drive.turn(Math.toRadians(60));
                        }
                        currentState = State.UNTURN;
                    }
                    break;
                case UNTURN:
                    if (!drive.isBusy()) {
                        currentState = State.BACKUP_FROM_SPIKES;
                        drive.followTrajectoryAsync(backUpFromSpikes);
                    }
                    break;
                case BACKUP_FROM_SPIKES:
                    if (!drive.isBusy()) {
                        if (propPos == PropFindLeft.pos.LEFT){
                            drive.followTrajectoryAsync(yellowDepositPathL);
                        } else if (propPos == PropFindLeft.pos.RIGHT){
                            drive.followTrajectoryAsync(yellowDepositPathR);
                        } else if (propPos == PropFindLeft.pos.MID){
                            drive.followTrajectoryAsync(yellowDepositPathC);
                        }

                        currentState = State.YELLOW_DEPOSIT_PATH;
                        //currentState = State.STOP;
                    } else {
                        target= CSCons.OuttakePosition.LOW_AUTO.getTarget();
                        drive.intakeToTransfer();
                        drive.outtakeToBackdrop();
                     }
                    break;

                case YELLOW_DEPOSIT_PATH:
                    if (!drive.isBusy() ) {
                        currentState = State.YELLOW_DEPOSIT_STRAIGHT;
                    } else {
//                        target= CSCons.OuttakePosition.LOW.getTarget();
//                        drive.intakeToTransfer();
//                        drive.outtakeToBackdrop();
                    }
                    break;
                case YELLOW_DEPOSIT_STRAIGHT:
                    if(!drive.isBusy()){
                        Trajectory straight= drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(3).build();
                        drive.followTrajectoryAsync(straight);

                        currentState=State.YELLOW_DEPOSIT;
                    }
                    break;
                case YELLOW_DEPOSIT:

                    if(resetInt == 0){
                        depositTime.reset();
                        resetInt++;
                    }
                    if(resetInt == 1){
                    if (!drive.isBusy()) {


                        //april tag alignment
                        //if april tag is aligned drop and
                        if (depositTime.milliseconds() > 500) {
                            drive.dropPixel();
                        }
                        if (depositTime.milliseconds() > 700) {
                            if (propPos== PropFindLeft.pos.LEFT){
                                drive.followTrajectoryAsync(parkL);
                            } else if (propPos== PropFindLeft.pos.RIGHT){
                                drive.followTrajectoryAsync(parkR);
                            } else{
                                drive.followTrajectoryAsync(parkC);
                            }
                            currentState = State.BACK;

                        }
                    }
                    }
                    break;
                case BACK:
                    if (!drive.isBusy()) {
                        drive.outtakeToTransfer();
                        target = 0;
                        park = drive.trajectoryBuilder(drive.getPoseEstimate(),false)
                                .splineToLinearHeading(new Pose2d(new Vector2d(50, 56), Math.toRadians(180)), Math.toRadians(0))
                                .build();
                        drive.followTrajectoryAsync(park);
                        currentState=State.PARK;
                    }
                break;
                case PARK:
                    if (!drive.isBusy()) {
                       // drive.outtakeToTransfer();
                       target=0;
                    }
                    break;
                case STOP:
                    break;
            }
        }
    }
}