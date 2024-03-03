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
public class ArmPIDFTestOpmode extends LinearOpMode {


    /*
    Imagine you have a robot with an arm that can move up and down.
    The goal is to make sure the arm stays in a specific position, say at a certain height,
    despite any disturbances or changes in weight it might experience.
    The PID Controller is the thing that helps with that.
    |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    Now, let's talk about the PID controller:
    |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    Proportional (P):

    Think of this as a "smartness" knob. If the arm is too low, the P part of the controller says,
    "Hey, the arm is too low, let's make it go up a bit." The farther away from the desired height,
    the stronger it pushes to correct the position.
    |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    Integral (I):

    Imagine the I part as a memory. It keeps track of how long the arm has been away from the desired position.
    If the arm has been too low for a while, the I part says, "Okay, we've been low for a bit too long, let's push a bit harder to fix it."
    |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    Derivative (D):

    The D part is like a quick thinker. If the arm is moving too fast, it says, "Hold on, we're moving too quickly, let's slow down a bit." It helps prevent overshooting the desired position.
    So, the PID controller is like having three friends (P, I, and D) working together to keep the robot arm exactly where it's supposed to be.
    |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    For the FTC robot arm:

    P looks at how far the arm is from the desired position.
    I remembers if the arm has been away from the desired position for a while.
    D keeps the arm from moving too fast.
    Together, they make sure the robot arm stays steady and reaches the right height,
    even if there's a sudden change in weight or if the robot gets bumped.

    (thanks chatgpt for an easy explanation on what pid is)
     */
    private PIDController armPIDLeft;
    private PIDController armPIDRight;

    /*
    these are your PID coefficients:

    (important: during the tuning process, with the chance of 99.99999999999999999999% you won't touch the I coefficient)

    scroll down a bit for explanation on how to find out ("tune") those values (including F coefficient):
     */

    public static double armP = 0, armI = 0, armD = 0;

    /*
    This F coefficient is, perhaps, the easiest but the most important part of your power controller.

    This coefficient helps the motor to counteract gravity, so your arm will not fall down under its own weight.
    As simple as that.

     */
    public static double armF = 0;

    //this is the target you tell your motors to achieve:

    public static int target = 0;

    //this is how many ticks your motor has to rotate for to spin approximately one degree:

    private static double ticks_in_degree = 288 / 360.0;


    /*
    So how do I tune the PID (F) coefficients?

    1) Connect from your PC (or laptop) to your Control Hub's Wifi
    2) Go to http://192.168.43.1:8080/dash
    3) In the left part of your screen, find a menu with a "Op Mode" drop list
    4) Select this ("ArmPIDFTestOpMode") OpMode, press "Init" and then "Start"
    5) In the middle menu (Graph), click on the icon in the top right corner
    6) Select the "target", "leftPos", and "rightPos" variables to be graphed
    7) In the rightmost menu (Configuration), select this opmode and here you will see all the changeable variables
    |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    Now let's first tune the F coefficient:

    1) Set your F coefficient equal to 0.001 (click on it in the Configuration menu, change the value and hit "Enter")
    2) Lift your arm up (manually. yes, with your bare hands) and release it
    3) If on the graph the lines of "leftPos" (and "rightPos") and "target" don't align at all
    (or, simply, if your arm just falls down :P), multiply the F coefficient by 10. (-> 0.01)
    4) If the arm falls again, from now on carefully increase the F coefficient by 0.01 each time.

    4.5) If the arm goes higher than you set it, decrease the F coefficient by 0.005, if the arm still goes higher than needed, decrease it again.
    If the arm falls down, increase the F by 0.005, if it still falls down, increase it again.
    If after increasing it for the second time it goes higher (or falls down),
    increase/decrease factor (0.01, 0.005, etc.) should be halved for more precise tuning.

    5) If the arm holds steady... congrats! Now the most important thing is to actually CHANGE THE VARIABLE IN THIS OPMODE
    (it won't be saved in the FTC Dashboard)

    Generally, the F coefficient should not be higher than 0.1 (but each case is individual).
    |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
    Now let's tune the P coefficient:

    1) Set the P to 0.001
    2) Set the target to an arbitrary value (say 1000, 500, 700, etc. but generally not bigger than 2500-3000)
    3) Watch how the "leftPos" ("rightPos") behave relatively to "target" graphs:
    3.1) If the pos graph is too low compared to the target, increase the P by another 0.001
    3.2) repeat step 3.1 if needed, maybe multiple times. while setting the position to something lower than current position
    (or some extremely low values like 100, 200)
    3.3) If the pos graph is overshooting too much (you will notice that dont worry - a VERY BIG bump in the pos graph),
    decrease the P by 0.0005 (then you can follow the same "halving" logic as in F coefficient)
    3.4) If the pos graph is overshooting  when it reaches the target (say, the difference is around 30-50 ticks), time to introduce a D coefficient:
    4) If everything is ok, change the variable to the value from Dashboard in this Opmode.
    |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

    Now let's tune the D coefficient:

    1) Set the D to 0.0001
    2) Set the target to something at least 500 ticks greater or lower than your current position
    3) Watch how the "leftPos" ("rightPos") behave relatively to "target" graphs:
    3.1) If there is still a big "bump" in the graph of pos when it reaches the target, increase the D by 0.00001
    3.2) repeat step 3.1 if needed
    3.3) If the target is being reached too slowly (then with a D = 0 for example), decrease the D by 0.00001
    3.4) repeat step 3.3 if needed
    3.5) follow the same "halving" logic as in tuning the F coefficient
    4) If everything is ok, change the variable to the value from Dashboard in this Opmode.
    |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

    Tuning the I coeffici.... we won't do it. No need. Absolutely. Don't ever touch it. Please.

    Now that you've done tuning here, change the values for P, I, D and F in the TeleOpMain.

    For a video guide on tuning:
    https://youtu.be/E6H6Nqe6qJo?t=441
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

        Motor.Encoder leftEncoder = armMotorLeft.encoder;
        leftEncoder = armMotorRight.encoder;


//        armMotors.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        armMotors.setRunMode(Motor.RunMode.RawPower);
        armMotors.stopAndResetEncoder();

        armPIDLeft = new PIDController(armP,armI,armD);
        armPIDRight = new PIDController(armP,armI,armD);

        armPIDLeft.setPID(armP, armI, armD);
        armPIDRight.setPID(armP, armI, armD);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            int leftPos = armMotorLeft.getCurrentPosition();
            int rightPos = armMotorRight.getCurrentPosition();


            double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * armF;

            double leftPower = armPIDLeft.calculate(leftPos, target) + ff;
            double rightPower = armPIDRight.calculate(rightPos, target) + ff;

            armMotorLeft.set(leftPower);
            armMotorRight.set(rightPower);



            telemetry.addData("Left pos:", leftPos);
            telemetry.addData("Right pos: ", rightPos);
            telemetry.addData("Target: ", target);
            telemetry.update();
        }
    }
}
