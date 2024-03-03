package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ArmPIDFTestOpmode extends LinearOpMode {

    private PIDController armPIDLeft;
    private PIDController armPIDRight;
    public static double armP = 0, armI = 0, armD = 0;
    public static double armF = 0;

    public static int target = 0;

    private static double ticks_in_degree = 288 / 360.0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Motor armMotorLeft;
        Motor armMotorRight;

        armMotorLeft = new Motor(hardwareMap, "armMotorLeft", 288, 125);
        armMotorRight = new Motor(hardwareMap, "armMotorRight", 288, 125);

        armMotorRight.setInverted(true);
        MotorGroup armMotors = new MotorGroup(armMotorLeft, armMotorRight);

        Motor.Encoder leftEncoder = armMotorLeft.encoder;
        leftEncoder = armMotorRight.encoder;


//        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        armMotors.setRunMode(Motor.RunMode.RawPower);
        armMotors.stopAndResetEncoder();

        armPIDLeft.setPID(armP, armI, armD);
        armPIDRight.setPID(armP, armI, armD);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            int leftPos = armMotorLeft.getCurrentPosition();
            int rightPos = armMotorRight.getCurrentPosition();


            double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * armF;

            double leftPower = armPIDLeft.calculate(leftPos, target) + ff;
            double rightPower = armPIDRight.calculate(rightPos, target) + ff;

            armMotorLeft.set(leftPower);
            armMotorRight.set(rightPower);

            if(leftPos <= 0) armMotorLeft.resetEncoder();
            if(rightPos <=0) armMotorRight.resetEncoder();

            telemetry.addData("Left pos:", leftPos);
            telemetry.addData("Right pos: ", rightPos);
            telemetry.addData("Target: ", target);
            telemetry.update();
        }
    }
}
