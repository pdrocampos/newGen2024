package org.firstinspires.ftc.teamcode.TestingComponentsPedro.PIDController;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "TestingPIDController", group = "PID")
public class TestingPIDController extends LinearOpMode {

    DcMotorEx motor;
    double integralSum = 0;
    double kp = 0;
    double ki = 0;
    double kd = 0;
    double kf = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException{
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while(opModeIsActive()){
            double power = PIDControl(1000,motor.getVelocity());
            motor.setPower(power);
        }
    }

    public double PIDControl(double reference, double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * kp) + (derivative * kd) + (integralSum * ki) + (reference * kf);
        return output;
    }
}
