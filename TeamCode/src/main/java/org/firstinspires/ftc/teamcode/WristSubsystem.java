package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.WristSubsystem.WristMode.OPEN;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristSubsystem {
    public enum WristMode {
        OPEN (0),
        CLOSE (1);


        WristMode(double position) {
            this.position = position;
        }

        public double getPosition() {
            return this.position;
        }

        private final double position;
    }

    private Servo wrist;
    private WristMode mode= OPEN;
    double position=0.8;
    //    private  final int offset=-1540;
    public WristSubsystem(HardwareMap hardwareMap) {
        wrist = hardwareMap.get(Servo.class, "wrist");

    }
    public void incrementAdd() {
        if (position<0.8){
            position+=0.1;
        }
        update();

    }

    public void incrementSubtract() {
        if (position>=0.47){
            position-=0.1;
        }

        update();
    }
    public void update() {
        wrist.setPosition(position);
    }
    public void reset(double goTo) {
        position=goTo;
        update();
    }


    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Wrist Position",wrist.getPosition());
    }
}
