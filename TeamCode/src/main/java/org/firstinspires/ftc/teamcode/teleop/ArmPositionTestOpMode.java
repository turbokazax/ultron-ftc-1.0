package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ArmPositionTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Motor armMotorLeft;
        Motor armMotorRight;

        armMotorLeft = new Motor(hardwareMap, "armMotorLeft", 288, 125);
        armMotorRight = new Motor(hardwareMap, "armMotorRight", 288, 125);

        armMotorRight.setInverted(true);
        MotorGroup armMotors = new MotorGroup(armMotorLeft, armMotorRight);
        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        armMotors.setRunMode(Motor.RunMode.RawPower);


        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            double leftPos = armMotorLeft.getCurrentPosition();
            double rightPos = armMotorRight.getCurrentPosition();
            telemetry.addData("Current left motor pos:", leftPos);
            telemetry.addData("Current right motor pos: ", rightPos);
            telemetry.update();
        }
    }
}
