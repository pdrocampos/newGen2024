package org.firstinspires.ftc.teamcode.TestingComponentsPedro.Teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "PreProgFieldCentric", group = "teleopImu")
public class PreProgFieldCentric extends LinearOpMode {
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

        Deadline gamepadRateLimit = new Deadline(500, TimeUnit.MILLISECONDS);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        waitForStart();
        while(opModeIsActive()){
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            double max = Math.max(Math.abs(strafe) + Math.abs(drive) + Math.abs(rotate), 1);

            if (gamepadRateLimit.hasExpired() && gamepad1.a) {
                imu.resetYaw();
                gamepadRateLimit.reset();
            }

            double drivePower = 0.8 - (0.6 * gamepad1.right_trigger);

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = -drive * Math.sin(heading) + strafe * Math.cos(heading);
            double adjustedLy = drive * Math.cos(heading) + strafe * Math.sin(heading);

            leftFront.setPower(((adjustedLy + adjustedLx + rotate) / max) / DRIVE_SENSIBILITY);
            leftBack.setPower(((adjustedLy - adjustedLx + rotate) / max) / DRIVE_SENSIBILITY);
            rightFront.setPower(((adjustedLy - adjustedLx - rotate) / max) / DRIVE_SENSIBILITY);
            rightBack.setPower(((adjustedLy + adjustedLx - rotate) / max) / DRIVE_SENSIBILITY);

            //double frontLeftPower = drive + strafe + rotate;
            //double frontRightPower = drive - strafe - rotate;
            //double backLeftPower = drive - strafe + rotate;
            //double backRightPower = drive + strafe - rotate;

            //leftFront.setPower(frontLeftPower);
            //rightFront.setPower(frontRightPower);
            //leftBack.setPower(backLeftPower);
            //rightBack.setPower(backRightPower);


        }
    }
}
