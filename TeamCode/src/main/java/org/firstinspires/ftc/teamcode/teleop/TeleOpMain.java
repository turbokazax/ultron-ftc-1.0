package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TeleOpMain extends OpMode {

    /*
     * Here i'm using the FTCLib's motors, servos, etc. instead of the default ones.
     * They're very easy to handle than the default ones
     * and FTCLib overall is the best library for teleOp programming (honestly)
     * if you wanna read more about it -> https://docs.ftclib.org/ftclib/   (ctrl+left click to open the link)
     *
     * if you need consultation for anything from this code text me in dm.
     * PLEASE read this code upside-down VERY carefully because I tried to include every possible
     * useful piece of information and thoroughly explain it.
     *
     * Btw, these gray lines of text you see in android studio are comments, they won't affect your code.
     */


    /*
     * These are the two motors you have on your drivebase.
     * VERY IMPORTANT!!!!!!!!!!!!:
     * Assume that the front of your robot is where the claw would be.
     * Motors are named respectively.
     */
    private Motor leftMotor;
    private Motor rightMotor;


    /*
    This is our drivebase. It's called differential because each side is controlled independently of another side.
    We will use it further in code to simplify the driving programming.
     */
    private DifferentialDrive drive;

    /*
     * Here, put the number of the ratio you use on your drivebase motors (for example:
     * if you have gears with ratios 3:1, 5:1, the total ratio would be 15:1, just add the numbers up)
     *
     * All "final" variables are so because there is no need to change them further in the code.
     * It prevents any possible change of this variable by crashing the program if you try to do so.
     */
    private final int DRIVE_MOTOR_RATIO = 4; //(4:1 CHANGE IT IF IT'S DIFFERENT)

    /*
     * Ticks Count Per Rev (CPR). What tick is? When the inner part of the motor (that is hidden by casing) rotates,
     * it triggers a magnetic detector inside it (encoder) multiple times per one complete rotation.
     * (For rev motors you use it is 28 ticks per INNER revolution, that's why for OUTER revolutions i multiply the RATIO by 28)
     * This is used to track the motors data (like its velocity, current Encoder Position and etc.)
     * Data is then retrieved using the Encoder cable (without it you won't be able to access this data).
     *<p></p>
     * Encoder position is the current number of ticks the motor has rotated. If motor rotates forward (or backwards, if motor was
     * intentionally inverted (it means that the default direction would be backward rotation, not forward),
     * encoder position increases and vice versa.
     *
     * Essentially, it describes how many revolutions will motor make and in what direction
     * (e.g +(28*4) ticks will mean one complete revolution for a motor with gear ratio 4:1, -(28*4)
     * will be one complete BACKWARD revolution)
     * ***************
     */
    private final int DRIVE_MOTOR_CPR = 28 * DRIVE_MOTOR_RATIO;

    /*
     * RPM (revolutions per minute) is simply the number of complete revolutions a motor could do in one minute.
     * The default number of revolutions per minute for inner motor is 6000, (for REV motors)
     * that's why for OUTER rotations it would be 6000 / RATIO revolutions
     * per minute
     */
    private final int DRIVE_MOTOR_RPM = 6000 / DRIVE_MOTOR_RATIO;

    /*
     * Motors that are used to rotate the arm:
     */
    private Motor armMotorLeft;
    private Motor armMotorRight;


    //

    /*
     * Here, i declare the gamepad for driver. GamepadEx from FTCLib is the enhanced handler of default gamepads and it's the best.
     */
    private GamepadEx driverOp;
    /*
     * Here, i declare the gamepad for scorer (if you have two drivers in your team, it's better to split so one would only drive and another one
     * would collect and score pixels)
     *
     * IMPORTANT: if you PLAN to use the second driver (scorer), please change the "driveOp" to "scorerOp" everywhere where i will mark further in the code.
     */
    private GamepadEx scorerOp;

    /*
     * These are 2 encoder positions for our arm to rotate to.
     * The former is the position when arm is low near the ground (and we assume it's our starting position)
     * The latter is the position of arm when we score the pixels on the backdrop.

     * IMPORTANT:
     * How to get these positions?

     * goto ArmPositionTestOpMode for explanation
     */
    private final int ARM_POSITION_LOW = 0;
    private final int ARM_POSITION_SCORE = 0;

    private int ARM_TARGET = 0;

    //goto ArmPIDFTestOpMode for explanation on these things:
    private PIDController armPIDLeft;
    private PIDController armPIDRight;
    public static double armP = 0, armI = 0, armD = 0;
    public static double armF = 0;

    private static double ticks_in_degree = 288 / 360.0;

    private boolean isArmSetToScoringPosition = false;

    //Claw servos:

    private ServoEx leftServo;
    private ServoEx rightServo;

    private boolean isClawOpened = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //here i initialize the driverOp gamepad assuming that the driver will use the first gamepad.
        driverOp = new GamepadEx(gamepad1);
        //comment (add the backslashes in the beginning of) the next line if you have the second driver (scorer):
        scorerOp = new GamepadEx(gamepad2);


        /*
         * IMPORTANT!!!!
         * IN THE SECOND ARGUMENT IN new Motor(...) (id) PUT IN THE ID YOU HAVE IN YOUR CONFIGURATION FROM DRIVER HUB FOR THOSE MOTORS
         * please don't mess it up
         */
        leftMotor = new Motor(hardwareMap, "leftMotor", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);
        rightMotor = new Motor(hardwareMap, "rightMotor", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);
        //Here, CPR and RPM were taken from the official REV site, no need to worry about these numbers:
        armMotorLeft = new Motor(hardwareMap, "armMotorLeft", 288, 125);
        armMotorRight = new Motor(hardwareMap, "armMotorRight", 288, 125);
        MotorGroup armMotors = new MotorGroup(armMotorLeft, armMotorRight);
        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        /*
        * Raw power means no internal controller does any work and the power supplied we will
        * calculate ourselves further in the code, depending on the desired position.
         */
        armMotors.setRunMode(Motor.RunMode.RawPower);

        /*
        *  because the default motor rotation (on the left side) is forward, the default rotation direction on the right side is backwards.
        * to account for that we invert the right motor so it's default rotation direction would be forward.
         */
        rightMotor.setInverted(true);
        armMotorRight.setInverted(true);


        // here i simply group two motors into one MotorGroup (from FTCLib) so i could handle them both at once in following two methods:
        MotorGroup driveMotors = new MotorGroup(leftMotor, rightMotor);
        /*
        * Velocity control run mode is where the speed of rotation of motors is handled by the inner motor controller.
        * As you use drive encoders, this would make the programming process MUCH easier because you would not need to account for any
        * wacky things with motors' speeds' you could've encountered if you didn't use them
         */
        driveMotors.setRunMode(Motor.RunMode.VelocityControl);
        /*
        This simply tells our motors to hold the robot on one place
        (i.e "brake" by giving negative power when any positive power is forcefully given)
        if zero power is supplied to them. This helps to prevent
        unnecessary drift that could happen if motors are given zero power while robot was moving
        and it drove a small distance before completely stopping.
         */
        driveMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        /*
        Initializing our drivebase with two motors we have:
         */
        drive = new DifferentialDrive(leftMotor, rightMotor);
        /*
        I did it because we have already inverted the right motor, so no need to invert it again:
         */
        drive.setRightSideInverted(false);

        armPIDLeft = new PIDController(armP, armI, armD);
        armPIDRight = new PIDController(armP, armI, armD);

        armPIDLeft.setPID(armP, armI, armD);
        armPIDRight.setPID(armP, armI, armD);

        //
        leftServo = new SimpleServo(hardwareMap, "leftServo", -135, 135);
        rightServo = new SimpleServo(hardwareMap, "rightServo", -135, 135);
        rightServo.setInverted(true);
    }


    /*
     * This is the method to handle driving. In my practice, splitting up handling different mechanisms into
     * different functions and then calling them in loop() is the best practice.
     */
    private void updateDrive() {
        /*
        here we get the inputs from our gamepads. Basically how gamepad's sticks work is that the amount of VERTICAL (that's why it's Y)
        displacement of the stick from center defines the current value driverOp.getLeftY().

        Put simply: the further you push the stick upwards or downwards, the faster the left or right side will drive.
         */

        //this one controls the left side (left motor):
        double leftPower = driverOp.getLeftY();
        //this one controls the right side (right motor):
        double rightPower = driverOp.getRightY();

        /*
         Here we use a tankDrive method for our Differential Drive. Tank drive is called like that because our drivebase...
         well...resembles the drivebase of a tank. All the difficult math of how to stabilize driving is hidden inside of this method
         so using it is pretty much a great shortcut.
        */
        drive.tankDrive(leftPower, rightPower);
    }

    private void updateArm() {
        int leftPos = armMotorLeft.getCurrentPosition();
        int rightPos = armMotorRight.getCurrentPosition();

        if (leftPos <= 0) armMotorLeft.resetEncoder();
        if (rightPos <= 0) armMotorRight.resetEncoder();

        //TODO: write comments on this part
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
        //TODO: finish this part
        if(scorerOp.wasJustPressed(GamepadKeys.Button.A))
            isClawOpened = !isClawOpened;

        if(isClawOpened){
            leftServo.setPosition(0.3);
            rightServo.setPosition(0.3);
        }else{
            leftServo.setPosition(0.51);
            rightServo.setPosition(0.51);
        }
    }

    @Override
    public void loop() {
        /*
         * As loop occurs basically forever, while your robot is running (when you pressed START and didn't press STOP on driver hub),
         * every function called there would be updated almost instantaneously (with a veeeery small delay of milliseconds (don't worry about it))
         */


        /*
         * this method is used to constantly track the buttons that are pressed (or held or whatever) on the gamepad.
         * It must be in the main loop() method.
         */
        driverOp.readButtons();
        scorerOp.readButtons();

        //handling the driving:
        updateDrive();
        //handling the arm:
        updateArm();
        //handling the claw:
        updateClaw();
    }
}
