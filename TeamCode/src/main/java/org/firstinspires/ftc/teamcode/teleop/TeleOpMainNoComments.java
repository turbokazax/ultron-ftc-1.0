package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpMainNoComments extends OpMode {

    private Motor leftMotor;
    private Motor rightMotor;
    private DifferentialDrive drive;
    private final int DRIVE_MOTOR_RATIO = 4;
    private final int DRIVE_MOTOR_CPR = 28 * DRIVE_MOTOR_RATIO;
    private final int DRIVE_MOTOR_RPM = 6000 / DRIVE_MOTOR_RATIO;

    private Motor armMotorLeft;
    private Motor armMotorRight;
    private GamepadEx driverOp;
    private GamepadEx scorerOp;
    private final int ARM_POSITION_LOW = 0;
    private final int ARM_POSITION_SCORE = 0;
    private int ARM_TARGET = 0;
    private PIDController armPIDLeft;
    private PIDController armPIDRight;
    public static double armP = 0, armI = 0, armD = 0;
    public static double armF = 0;
    private static double ticks_in_degree = 288 / 360.0;
    private boolean isArmSetToScoringPosition = false;

    private ServoEx leftServo;
    private ServoEx rightServo;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        driverOp = new GamepadEx(gamepad1);
        scorerOp = new GamepadEx(gamepad2);

        leftMotor = new Motor(hardwareMap, "leftMotor", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);
        rightMotor = new Motor(hardwareMap, "rightMotor", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);
        armMotorLeft = new Motor(hardwareMap, "armMotorLeft", 288, 125);
        armMotorRight = new Motor(hardwareMap, "armMotorRight", 288, 125);
        MotorGroup armMotors = new MotorGroup(armMotorLeft, armMotorRight);
        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        armMotors.setRunMode(Motor.RunMode.RawPower);
        rightMotor.setInverted(true);
        armMotorRight.setInverted(true);

        MotorGroup driveMotors = new MotorGroup(leftMotor, rightMotor);
        driveMotors.setRunMode(Motor.RunMode.VelocityControl);
        driveMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        drive = new DifferentialDrive(leftMotor, rightMotor);
        drive.setRightSideInverted(false);

        armPIDLeft = new PIDController(armP, armI, armD);
        armPIDRight = new PIDController(armP, armI, armD);
        armPIDLeft.setPID(armP, armI, armD);
        armPIDRight.setPID(armP, armI, armD);
    }

    private void updateDrive() {
        double leftPower = driverOp.getLeftY();
        double rightPower = driverOp.getRightY();
        drive.tankDrive(leftPower, rightPower);
    }

    private void updateArm() {
        int leftPos = armMotorLeft.getCurrentPosition();
        int rightPos = armMotorRight.getCurrentPosition();

        if (leftPos <= 0) armMotorLeft.resetEncoder();
        if (rightPos <= 0) armMotorRight.resetEncoder();

        if (scorerOp.wasJustPressed(GamepadKeys.Button.Y)) {
            isArmSetToScoringPosition = !isArmSetToScoringPosition;
        }

        double ff;
        double leftPower;
        double rightPower;
        if (isArmSetToScoringPosition) {
            ARM_TARGET = ARM_POSITION_SCORE;
        } else {
            ARM_TARGET = ARM_POSITION_LOW;
        }

        ff = Math.cos(Math.toRadians(ARM_TARGET / ticks_in_degree)) * armF;
        leftPower = armPIDLeft.calculate(leftPos, ARM_TARGET) + ff;
        rightPower = armPIDRight.calculate(rightPos, ARM_TARGET) + ff;
        armMotorLeft.set(leftPower);
        armMotorRight.set(rightPower);
    }

    private void updateClaw() {

    }

    @Override
    public void loop() {
        driverOp.readButtons();
        scorerOp.readButtons();
        updateDrive();
        updateArm();
        updateClaw();
    }
}