package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


public class TeleOpMain extends OpMode {

    /*
     * Here i'm using the FTCLib's motors, servos, etc. They're very easy to handle than the default ones
     * and FTCLib overall is the best library for teleOp programming (honestly)
     * if you wanna read more about it -> https://docs.ftclib.org/ftclib/   (ctrl+left click to open the link)
     *
     * if you need consultation for anything from this code text me in dm.
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
     */
    private final int DRIVE_MOTOR_RATIO = 4; //(4:1)

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

    private Motor armMotor;


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

    @Override
    public void init() {
        //here i initialize the driverOp gamepad assuming that the driver will use the first gamepad.
        driverOp = new GamepadEx(gamepad1);
        //uncomment (remove the backslashes) the next line if you have the second driver (scorer):
//        scorerOp = new GamepadEx(gamepad2)


        /*
        IMPORTANT!!!!
         IN THE SECOND ARGUMENT IN new Motor(...) (id) PUT IN THE ID YOU HAVE IN YOUR CONFIGURATION FROM DRIVER HUB FOR THOSE MOTORS
         please don't mess it up
         */
        leftMotor = new Motor(hardwareMap, "leftMotor", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);
        rightMotor = new Motor(hardwareMap, "rightMotor", DRIVE_MOTOR_CPR, DRIVE_MOTOR_RPM);


        /*
        because the default motor rotation (on the left side) is forward, the default rotation direction on the right side is backwards.
        to account for that we invert the right motor so it's default rotation direction would be forward.
         */
        rightMotor.setInverted(true);


        // here i simply group two motors into one MotorGroup (from FTCLib) so i could handle them both at once in following two methods:
        MotorGroup driveMotors = new MotorGroup(leftMotor, rightMotor);
        /*
        Velocity control run mode is where the speed of rotation of motors is handled by the inner motor contoller.
        As you use drive encoders, this would make the programming process MUCH easier because you would not need to account for any
        wacky things with motors' speeds' you could've encountered if you didn't use them
         */
        driveMotors.setRunMode(Motor.RunMode.VelocityControl);
        /*
        This simply tells our motors to hold the robot on one place if zero power is supplied to them. This helps to prevent
        unnecessary drift that could happen if motors are given zero power while robot was moving
        and it drove a small distance before completely stopping.
         */
        driveMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }


    /*
     * This is the method to handle driving. In my practice, splitting up handling different mechanisms into
     * different functions and then calling them in loop() is the best practice.
     */
    private void updateDrive() {
        /*
        here we get the inputs from our gamepads. Basically how gamepad's sticks work is that the amount of VERTICAL (that's why it's Y)
        displacement of the stick from center defines the current value driverOp.getLeftY().

        Put simply: the further you push the stick upwards or downwards, the faster the left or right motor will rotate.
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
        //TODO: finish the arm code
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

        //handling the driving:
        updateDrive();
    }
}
