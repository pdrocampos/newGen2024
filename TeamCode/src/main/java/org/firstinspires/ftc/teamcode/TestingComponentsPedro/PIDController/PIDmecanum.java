package org.firstinspires.ftc.teamcode.TestingComponentsPedro.PIDController;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "MecanumPID")
public class PIDmecanum extends LinearOpMode {

    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;

    private BNO055IMU imu;

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;

    ElapsedTime tempo = new ElapsedTime();
    private double ultimoErro = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotorEx.class,"leftFront");
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFront = hardwareMap.get(DcMotorEx.class,"rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBack = hardwareMap.get(DcMotorEx.class,"leftBack");
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBack = hardwareMap.get(DcMotorEx.class,"rightBack");
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters parametro =new BNO055IMU.Parameters();
        parametro.mode = BNO055IMU.SensorMode.IMU;
        parametro.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parametro);

        waitForStart();

        double anguloReferencia = Math.toRadians(90);
        while (opModeIsActive()){

            double power = ControlePID(anguloReferencia, imu.getAngularOrientation().firstAngle);
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(-power);
            rightBack.setPower(-power);
        }
    }

    public double ControlePID(double referencia, double estado) {

        double erro = quebraAngulo(referencia - estado) ;
        integralSum += erro * tempo.seconds();
        double derivada = (erro - ultimoErro) / tempo.seconds();
        ultimoErro = erro;

        tempo.reset();

        double output = (erro * Kp) + (derivada * Kd) + (integralSum * Ki);

        return erro;
    }

    public double quebraAngulo (double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians <- Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }

}
