package org.firstinspires.ftc.teamcode.lm2COMPCODE.Teleop.Threads;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.lm2COMPCODE.Teleop.packages.SliderManger;
import org.firstinspires.ftc.teamcode.lm2COMPCODE.Teleop.packages.servoManger;
import org.firstinspires.ftc.teamcode.lm2COMPCODE.Teleop.Teleop;

public class gamepad2Controls extends Thread{
    public boolean running = true;
    public boolean resetEnabled = false;
    public boolean bypassEnabled = false;

    public SliderManger SM = new SliderManger();
    public DcMotor sc, sr;
    public servoManger clawServo = new servoManger();
    public servoManger clawRotateServo = new servoManger();
    public servoManger clawRotateServo2 = new servoManger();

    public Gamepad gamepad2;
    private Teleop mainFile;

    public void init(Teleop main){
        this.mainFile = main;
        this.gamepad2 = main.gamepad2;
        this.SM = main.SM;
        this.sc = main.sc;
        this.sr = main.sr;
        this.clawServo = main.clawServo;
        this.clawRotateServo = main.clawRotateServo;
        this.clawRotateServo2 = main.clawRotateServo2;
    }

    public void run(){
        try{
            while(running && !mainFile.isStopRequested()){
                bypassEnabled = gamepad2.back;
                resetEnabled = gamepad2.start;
                if(sc.getCurrentPosition() < -20 && sr.getCurrentPosition() > 600) {
                    SM.move(gamepad2.left_stick_y - 0.01 > 1.0 ? gamepad2.left_stick_y : gamepad2.left_stick_y - 0.01);
                }else{
                    if(sc.getCurrentPosition() > -1100) {
                        SM.move(gamepad2.left_stick_y);
                    }else{
                        if(gamepad2.left_stick_y > 0.0) {
                            SM.move(1.0);
                        }else{
                            SM.move(0.0);
                        }
                    }
                }
                if(-gamepad2.right_stick_y <= -0.3 && !gamepad2.back){
                    clawRotateServo.setServoPosition(0.4);
                    SM.setPos(0, 0.5);
                }
                if(-gamepad2.right_stick_y >= 0.3 && !gamepad2.back){
                    SM.setPos(675, 0.5);
                }
                if(gamepad2.b) {
                    clawRotateServo.setServoPosition(0.1);
                }
                if(gamepad2.a) {
                    clawRotateServo.setServoPosition(0.05);
                }
                if(gamepad2.y) {
                    clawRotateServo.setServoPosition(0.4);
                }
                if(gamepad2.x) {
                    clawRotateServo.setServoPosition(0.6);
                }
                if(gamepad2.right_bumper){
                    clawServo.setServoPosition(0.7);
                }
                if(gamepad2.left_bumper){
                    clawServo.setServoPosition(0.33);
                }
                if(gamepad2.dpad_left){
                    clawRotateServo2.setServoPosition(0.8);
                }else if(gamepad2.dpad_right){
                    clawRotateServo2.setServoPosition(0.3);
                }else if(gamepad2.dpad_up){
                    clawRotateServo2.setServoPosition(0.1);
                }else if(gamepad2.dpad_down){
                    clawRotateServo2.setServoPosition(0.9);
                }else{
                    clawRotateServo2.setServoPosition(0.5);
                }
            }
        }catch(Exception e){
            mainFile.telemetry.addLine("ERROR");
            mainFile.telemetry.addLine(String.valueOf(e));
        }
    }
}
