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
    /*
     IMPORTANT:
    So how to get these positions?

    The only way to do it without any excessive math is to do it empirically.
    Launch the ArmPositionTestOpMode and look on the values on your driver hub;
    Now, from the ground position (1) rotate the arm to the desired position
    in which the pixels will be scored (I assume here that you have a backdrop (e.g AndyMark's Centerstage Backdrop))
    (if not, just make sure the angle between the arm and horizontal is approximately 60 degrees).

    Now, remember this position and put it in the ARM_POSITION_SCORE variable in the TeleOpMain

    (1):
    PLEASE MAKE SURE THAT THIS GROUND POSITION IS THE POSITION
    THAT WILL BE ACTUALLY THE (STARTING) POSITION DURING THE GAME
    TO MAKE SURE NO UNCERTAINTIES WILL TAKE PLACE
     */

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
