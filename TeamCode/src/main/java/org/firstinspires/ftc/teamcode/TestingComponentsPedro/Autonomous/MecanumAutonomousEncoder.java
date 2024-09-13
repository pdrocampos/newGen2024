package org.firstinspires.ftc.teamcode.TestingComponentsPedro.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "MecanumAutonomousEncoder", group = "Autonomous")
public class MecanumAutonomousEncoder extends LinearOpMode {

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    static final double     COUNTS_PER_MOTOR_REV    = 28.0;
    static final double     DRIVE_GEAR_REDUCTION    = 18.8803;
    static final double     WHEEL_CIRCUMFERENCE_MM   = 96.0 * Math.PI;
    static final double     COUNTS_PER_WHEEL_REV    = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double     COUNTS_PER_CM           = (COUNTS_PER_WHEEL_REV / WHEEL_CIRCUMFERENCE_MM) * 10;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class,"motorLeftFront");
        rightFront = hardwareMap.get(DcMotorEx.class,"motorRightFront");
        leftBack = hardwareMap.get(DcMotorEx.class,"motorLeftBack");
        rightBack = hardwareMap.get(DcMotorEx.class,"motorRightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        configureStopResetEncoder(); //Configura o motor para reiniciar os encoders

        waitForStart();

        configureStopResetEncoder();
        setPositionDrive(1,20);
        setVelocity(175,175,175,175);

        configureStopResetEncoder();
        setPositionStrafe(1,20);
        setVelocity(175,175,175,175);

        configureStopResetEncoder();
        setPositionStrafe(-1,20);
        setVelocity(175,175,175,175);

        configureStopResetEncoder();
        setPositionDrive(-1,20);
        setVelocity(1751,175,175,175);

        while(opModeIsActive()){
            telemetry.addData("EncoderFront",  " left: %7d  ----  right: %7d", leftFront.getCurrentPosition(), rightFront.getCurrentPosition());
            telemetry.addData("EncoderRight",  " left: %7d  ----  right: %7d", leftBack.getCurrentPosition(), rightBack.getCurrentPosition());
            telemetry.update();
        }
    }

    //METODOS PARA SIMPLIFICAÇÃO DO CÓDIGO
    public void configureStopResetEncoder(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void setPositionDrive(int x, int distancia){ //y pode assumir 1 ou -1 pra definir a direção movimento
        if(x == 1){
            leftFront.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
            rightFront.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
            leftBack.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
            rightBack.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
        }else{
            distancia=-distancia;
            leftFront.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
            rightFront.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
            leftBack.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
            rightBack.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
        }
    }
    public void setPositionStrafe(int x, int distancia){ //x pode assumir 1 ou -1 para definir a direção do movimento
        if(x == 1){
            leftFront.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
            rightFront.setTargetPosition((int) ((int) -distancia * COUNTS_PER_CM));
            leftBack.setTargetPosition((int) ((int) -distancia * COUNTS_PER_CM));
            rightBack.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
        }else{
            leftFront.setTargetPosition((int) ((int) -distancia * COUNTS_PER_CM));
            rightFront.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
            leftBack.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
            rightBack.setTargetPosition((int) ((int) -distancia * COUNTS_PER_CM));
        }
    }
    public void setPositionDiagonal(int x, int y, int distancia){
        if(x == 1){
            if(y == 1){
                leftFront.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
                rightFront.setTargetPosition(0);
                leftBack.setTargetPosition(0);
                rightBack.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
            }else{
                leftFront.setTargetPosition(0);
                rightFront.setTargetPosition((int) ((int) -distancia * COUNTS_PER_CM));
                leftBack.setTargetPosition((int) ((int) -distancia * COUNTS_PER_CM));
                rightBack.setTargetPosition(0);
            }
        }else{
            if(y == 1){
                leftFront.setTargetPosition(0);
                rightFront.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
                leftBack.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
                rightBack.setTargetPosition(0);
            }else{
                leftFront.setTargetPosition((int) ((int) -distancia * COUNTS_PER_CM));
                rightFront.setTargetPosition(0);
                leftBack.setTargetPosition(0);
                rightBack.setTargetPosition((int) ((int) -distancia * COUNTS_PER_CM));
            }
        }
    }
    public void setPositionCurve(int x, int distancia){
        if(x == 1){
            leftFront.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
            rightFront.setTargetPosition((int) ((int) -distancia * COUNTS_PER_CM));
            leftBack.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
            rightBack.setTargetPosition((int) ((int) -distancia * COUNTS_PER_CM));
        }else{
            distancia=-distancia;
            leftFront.setTargetPosition((int) ((int) -distancia * COUNTS_PER_CM));
            rightFront.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
            leftBack.setTargetPosition((int) ((int) -distancia * COUNTS_PER_CM));
            rightBack.setTargetPosition((int) ((int) distancia * COUNTS_PER_CM));
        }
    }
    public void configureModeEncoder(DcMotor.RunMode mode ){
        leftFront.setMode(mode);
        rightFront.setMode(mode);
        leftBack.setMode(mode);
        rightBack.setMode(mode);
    }
    public void setVelocity(int velocityLF, int velocityRF, int velocityLB, int velocityRB){
        configureModeEncoder(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setVelocity((velocityLF/60) * COUNTS_PER_WHEEL_REV);
        rightFront.setVelocity((velocityRF/60) * COUNTS_PER_WHEEL_REV);
        leftBack.setVelocity((velocityLB/60) * COUNTS_PER_WHEEL_REV);
        rightBack.setVelocity((velocityRB/60) * COUNTS_PER_WHEEL_REV);
        while((leftFront.isBusy() && rightFront.isBusy() && rightBack.isBusy() && rightBack.isBusy())){
        }
    }
}
