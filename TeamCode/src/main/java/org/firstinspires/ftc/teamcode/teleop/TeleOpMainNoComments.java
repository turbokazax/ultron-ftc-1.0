package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class TeleOpMainNoComments extends OpMode {

    /*
    This is a clean version of your Main TeleOp but without all the unnecessary comments & explanation
     */
    private Motor leftMotor;
    private Motor rightMotor;

    private DifferentialDrive drive;

    private final int DRIVE_MOTOR_RATIO = 4; //(CHANGE IT IF IT'S DIFFERENT)

    private final int DRIVE_MOTOR_CPR = 28 * DRIVE_MOTOR_RATIO;


    private final int DRIVE_MOTOR_RPM = 6000 / DRIVE_MOTOR_RATIO;


    private Motor armMotorLeft;
    private Motor armMotorRight;


    private GamepadEx driverOp;
    private GamepadEx scorerOp;

    @Override
    public void init() {

        driverOp = new GamepadEx(gamepad1);


        leftMotor = new Motor(hardwareMap, "leftMotor", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);
        rightMotor = new Motor(hardwareMap, "rightMotor", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);

        armMotorLeft = new Motor(hardwareMap, "armMotorLeft", 288, 125);
        armMotorRight = new Motor(hardwareMap, "armMotorRight", 288, 125);

        rightMotor.setInverted(true);
        armMotorRight.setInverted(true);


        MotorGroup driveMotors = new MotorGroup(leftMotor, rightMotor);

        driveMotors.setRunMode(Motor.RunMode.VelocityControl);

        driveMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        drive = new DifferentialDrive(leftMotor, rightMotor);

        drive.setRightSideInverted(false);
    }


    private void updateDrive() {


        double leftPower = driverOp.getLeftY();

        double rightPower = driverOp.getRightY();


        drive.tankDrive(leftPower, rightPower);
    }

    private void updateArm() {
        //TODO: finish the arm code
    }


    @Override
    public void loop() {

        driverOp.readButtons();


        updateDrive();
    }
}
