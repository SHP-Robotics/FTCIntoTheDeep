package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp
public class TestDrive extends LinearOpMode {
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

        waitForStart();

        frontLeft.setPower(0.4);
        sleep(2000);
        frontLeft.setPower(0.0);

        frontRight.setPower(0.4);
        sleep(2000);
        frontRight.setPower(0.0);

        backLeft.setPower(0.4);
        sleep(2000);
        backLeft.setPower(0.0);

        backRight.setPower(0.4);
        sleep(2000);
        backRight.setPower(0.0);
    }
}
