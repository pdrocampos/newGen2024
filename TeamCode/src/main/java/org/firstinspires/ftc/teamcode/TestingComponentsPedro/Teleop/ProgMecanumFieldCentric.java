package org.firstinspires.ftc.teamcode.TestingComponentsPedro.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
PROGRAMAÇÃO MECANUM FIELD-CENTER 2023/2024 FUNCIONAL!
*/

@TeleOp(name = "ProgMecanumFieldCentric", group = "TeleopImu")
public class ProgMecanumFieldCentric extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private static final double DRIVE_SENSIBILITY = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {

        leftFront = hardwareMap.get(DcMotor.class,"motorLeftFront");
        rightFront = hardwareMap.get(DcMotor.class,"motorRightFront");
        leftBack = hardwareMap.get(DcMotor.class,"motorLeftBack");
        rightBack = hardwareMap.get(DcMotor.class,"motorRightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive()){
            double drive = DRIVE_SENSIBILITY * gamepad1.left_stick_x;
            double strafe = DRIVE_SENSIBILITY * gamepad1.left_stick_y;
            double rotate = DRIVE_SENSIBILITY * gamepad1.right_stick_x;

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotate), 1);

            double drivePower = 0.8 - (0.6 * gamepad1.right_trigger);

            leftFront.setPower(((rotY + rotX + rotate) / denominator) / drivePower);
            leftBack.setPower(((rotY - rotX + rotate) / denominator) / drivePower);
            rightFront.setPower(((rotY - rotX - rotate) / denominator) / drivePower);
            rightBack.setPower(((rotY + rotX - rotate) / denominator) / drivePower);

        }
    }
}


