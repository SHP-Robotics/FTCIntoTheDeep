package org.firstinspires.ftc.teamcode.subsystems.Arm;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.pivotPIDConstants.*;
import static org.firstinspires.ftc.teamcode.subsystems.Arm.ArmConstants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PID.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.utils.PID.TrapezoidProfile;

import java.util.function.DoubleSupplier;

public class PivotSubsystem extends SubsystemBase {
    public DoubleSupplier armLength;
    public DoubleSupplier armCOM;
    public double currentArmCOM;
    public double currentArmAngle;
    HardwareMap map;
    public MotorEx pivotLeft;
    public MotorEx pivotRight;
    public Motor.Encoder leftEncoder;
    public Motor.Encoder rightEncoder;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public
    ProfiledPIDController m_pivotPID;



    public PivotSubsystem(HardwareMap map, DoubleSupplier armLength){
        this.map = map;
        pivotLeft = new MotorEx(map,"pivotLeft");//tbd
        pivotLeft.setInverted(true);
        pivotLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        pivotRight = new MotorEx(map,"pivotRight");//tbd
        pivotRight.setInverted(false);
        pivotRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.armLength = armLength;
        m_pivotPID = new ProfiledPIDController(pKP,pKI,pKD,new TrapezoidProfile.Constraints(vConstraint,aConstraint));
        m_pivotPID.m_controller.setMaximumAbsOutput(pMaxOutput);
        leftEncoder = pivotLeft.encoder;
        rightEncoder = pivotRight.encoder;
        leftEncoder.reset();
        rightEncoder.reset();

    }

    @Override
    public void periodic() {
        updateValues();
        updateTelemetry();
        if (m_pivotPID.atGoal()) {
            setMotors(calculateFeedForward());
        } else {
            setMotors(-m_pivotPID.calculate(currentArmAngle) + calculateFeedForward());
        }
    }

    
    private void updateValues() {
        calcArmAngle();
        m_pivotPID.setPID(pKP,pKI,pKD);
        m_pivotPID.setConstraints(new TrapezoidProfile.Constraints(vConstraint,aConstraint));
        m_pivotPID.setTolerance(pTolerance);
        m_pivotPID.setGoalTolerance(pGoalTolerance, pGoalVelocityTolerance);
        m_pivotPID.m_controller.setAccumilatorResetTolerance(pGoalTolerance);//TODO:look at this
        m_pivotPID.setIzone(pIzone);
        m_pivotPID.m_controller.setMaximumAbsOutput(pMaxOutput);

    }



    private void updateTelemetry() {
        dashboardTelemetry.addData("armAngle", currentArmAngle);
//        dashboardTelemetry.addData("COMAngle", aCOMAngle());
        dashboardTelemetry.addData("_pid+ff value", m_pivotPID.calculate(currentArmAngle)+calculateFeedForward());
        dashboardTelemetry.addData("_pid value", -m_pivotPID.calculate(currentArmAngle));
        dashboardTelemetry.addData("_FF", calculateFeedForward());
        dashboardTelemetry.addData("balanceAngle", aBalanceAngle());
        dashboardTelemetry.addData("kG", akG(armLength));
        dashboardTelemetry.addData("rightEncoder", rightEncoder.getPosition());
        dashboardTelemetry.addData("rightEncoder rev", rightEncoder.getRevolutions());
        dashboardTelemetry.addData("leftEncoder rev", leftEncoder.getRevolutions());
        dashboardTelemetry.update();

    }

    public double calculateFeedForward(){
        return -akG(armLength) * Math.cos(Math.toRadians(aCOMAngle()));
    }

    private double aCOMAngle(){
        return currentArmAngle+90 - aBalanceAngle();
    }

    private double aBalanceAngle(){
        return 3.59*armLength.getAsDouble() + 119;
    }
    private double akG(DoubleSupplier x) {
        return 0.0782 + -0.0121*x.getAsDouble() + 3.17E-03*Math.pow(x.getAsDouble(),2);
    }




    private void calcArmAngle() {
        currentArmAngle = 10.321*leftEncoder.getRevolutions();//linear equation (encoder to angle)
    }

    public void setMotors(double power){
        if(power>0.001)
        {
            pivotRight.set(power+kS);
            pivotLeft.set(power+kS);
        }
        else if(power<-0.001)
        {
            pivotRight.set(power-kS);
            pivotLeft.set(power-kS);

        }
        else{
            pivotRight.set(0);
            pivotLeft.set(0);
        }
    }
    public Command set(){
        return new InstantCommand(()->m_pivotPID.setGoal(pSetpoint,currentArmAngle),this);
    }
    public Command disablePID(){
        return new InstantCommand(()->m_pivotPID.disable());
    }
    public Command enablePID(){
        return new InstantCommand(()->m_pivotPID.enable(currentArmAngle));
    }




}