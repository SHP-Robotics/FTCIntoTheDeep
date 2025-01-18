package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;

@Config
public class PestoFTCConfig {
    public static double ODOMETRY_TICKS_PER_INCH = 505.3169;
    public static double FORWARD_OFFSET = -6.5;
    public static double ODOMETRY_WIDTH = 14;
    public static double DECELERATION = 2.3;
    public static double MAX_VELOCITY = 56;


    public static final DcMotorSimple.Direction leftEncoderDirection = REVERSE;
    public static final DcMotorSimple.Direction centerEncoderDirection = REVERSE;
    public static final DcMotorSimple.Direction rightEncoderDirection = REVERSE;

    public static String leftName = "frontLeft";
    public static String centerName = "backLeft";
    public static String rightName = "frontRight";

    public static final DcMotorSimple.Direction frontLeftDirection = REVERSE;
    public static final DcMotorSimple.Direction frontRightDirection = FORWARD;
    public static final DcMotorSimple.Direction backLeftDirection = REVERSE;
    public static final DcMotorSimple.Direction backRightDirection = FORWARD;

    public static String frontLeftName = "frontLeft";
    public static String frontRightName = "frontRight";
    public static String backLeftName = "backLeft";
    public static String backRightName = "backRight";

    public static MecanumController getMecanumController(HardwareMap hardwareMap) {
        MecanumController mecanumController = new MecanumController(hardwareMap, new String[] {
                frontLeftName,
                frontRightName,
                backLeftName,
                backRightName
        });

        mecanumController.configureMotorDirections(new DcMotorSimple.Direction[]{
                frontLeftDirection,
                frontRightDirection,
                backLeftDirection,
                backRightDirection
        });

//        mecanumController.setPowerVectors(new Vector2D[]{
//                Vector2D.scale(new Vector2D(57, 39), 1/69.0651865993),
//                Vector2D.scale(new Vector2D(-57, 39), 1/69.0651865993),
//                Vector2D.scale(new Vector2D(-57, 39), 1/69.0651865993),
//                Vector2D.scale(new Vector2D(57, 39), 1/69.0651865993)
//        });

        mecanumController.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanumController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        return mecanumController;
    }

    public static TeleOpController getTeleOpController(MecanumController mecanumController, DeterministicTracker tracker, HardwareMap hardwareMap) {
        TeleOpController teleOpController = new TeleOpController(mecanumController, hardwareMap);

        teleOpController.configureIMU(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );

        teleOpController.setSpeedController((gamepad) -> {

            return 1.0;
        });

        teleOpController.counteractCentripetalForce(tracker, MAX_VELOCITY);
//        teleOpController.deactivateCentripetalForce();

        return teleOpController;
    }

   public static DeterministicTracker getTracker(HardwareMap hardwareMap) {
        return new ThreeWheelOdometryTracker.TrackerBuilder(hardwareMap,
                ODOMETRY_TICKS_PER_INCH,
                FORWARD_OFFSET,
                ODOMETRY_WIDTH,
                leftName,
                centerName,
                rightName,
                leftEncoderDirection,
                centerEncoderDirection,
                rightEncoderDirection).build();
    }
}