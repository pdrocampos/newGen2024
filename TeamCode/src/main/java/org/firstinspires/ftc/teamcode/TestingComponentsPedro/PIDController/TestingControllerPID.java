package org.firstinspires.ftc.teamcode.TestingComponentsPedro.PIDController;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TestingPID", group = "PID")
public class TestingControllerPID extends LinearOpMode {
    private DcMotor motor;
    double kp = 1;
    double ki = 1;
    double kd = 1;
    int reference = 2000;
    double integralSum = 0;
    int lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class,"motorLeftBack");
        waitForStart();

        int value = motor.getCurrentPosition();
        int erro = 0;

        while(Math.abs(erro) != reference){
            int encoderPosition = motor.getCurrentPosition();
            erro = (reference - encoderPosition);
            double derivative = (erro - lastError) / timer.seconds();
            integralSum = integralSum + (erro * timer.seconds());
            double saida =  (kp * erro) + (ki * integralSum) + (kd * derivative);
            motor.setPower(saida);

            lastError = erro;

            timer.reset();
        }
    }
}
