package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Config
@TeleOp
public class ArmPositionTestOpMode extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Motor armMotorLeft;
        Motor armMotorRight;

        armMotorLeft = new Motor(hardwareMap, "armMotorLeft", 288, 125);
        armMotorRight = new Motor(hardwareMap, "armMotorRight", 288, 125);

        armMotorRight.setInverted(true);
        MotorGroup armMotors = new MotorGroup(armMotorLeft, armMotorRight);


//        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        armMotors.setRunMode(Motor.RunMode.RawPower);
        armMotors.stopAndResetEncoder();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            int leftPos = armMotorLeft.getCurrentPosition();
            int rightPos = armMotorRight.getCurrentPosition();
            telemetry.addData("Current left motor pos:", leftPos);
            telemetry.addData("Current right motor pos: ", rightPos);
            telemetry.update();
        }
    }
}
