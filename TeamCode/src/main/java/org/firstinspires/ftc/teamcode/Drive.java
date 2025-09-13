package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.drivebases.controllers.MecanumController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp
public class Drive extends LinearOpMode {
    @Override
    public void runOpMode() {
        MotorCortex.initialize(hardwareMap);

        CortexLinkedMotor frontLeft = MotorCortex.getMotor("frontLeft");
        CortexLinkedMotor frontRight = MotorCortex.getMotor("frontRight");
        CortexLinkedMotor backLeft = MotorCortex.getMotor("backLeft");
        CortexLinkedMotor backRight = MotorCortex.getMotor("backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        MecanumController mecanumController = new MecanumController(
                frontLeft,
                frontRight,
                backLeft,
                backRight
        );

        TeleOpController teleOpController = new TeleOpController(
                mecanumController, hardwareMap
        );

        DcMotor intake = MotorCortex.getMotor("intake");
        DcMotor outtake = MotorCortex.getMotor("outtake");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        outtake.setPower(0.5);

        while (opModeIsActive() && !isStopRequested()) {
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.right_trigger > 0.5) {
                intake.setPower(1.0);
                outtake.setPower(1.0);
            } else if (gamepad1.left_trigger > 0.5) {
                intake.setPower(-1.0);
                outtake.setPower(0.5);
            } else {
                intake.setPower(0.0);
                outtake.setPower(0.5);
            }
        }
    }
}
