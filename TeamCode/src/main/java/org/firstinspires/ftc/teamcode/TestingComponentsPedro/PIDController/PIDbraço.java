package org.firstinspires.ftc.teamcode.TestingComponentsPedro.PIDController;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
public class PIDbraço extends LinearOpMode {

    DcMotorEx motor;

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    private double lastError = 0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "motor");

        waitForStart();
        while (opModeIsActive()){
           double power = PIDControl(100, motor.getCurrentPosition());
           motor.setPower(power);
        }
    }

    public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return output;
    }
}
