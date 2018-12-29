package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name="autotest", group="Connection")

public class autonomousTest extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_Connection robot = new Hardware_Connection();
    private ElapsedTime     runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.1;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.15;     // Larger is more responsive, but also less stable
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private TFObjectDetector tfod;

    //private TFObjectDetector tfod;
    private enum GoldPosition {
        LEFT,
        RIGHT,
        MIDDLE
    }

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.fullReset();
        robot.fullEncoder();
        robot.fullEncoderReset();
        telemetry.addData("status:", "motors are ready");
        telemetry.update();

        while (!isStarted()) {
            telemetry.addData(">>>", "ready for start<<<");
            telemetry.update();
        }

        waitForStart();
        runtime.reset();
        robot.arm_motors(-1);
        robot.arm_opening_system.setPower(1);
        for(double armPower = -1; armPower != 0; armPower += 0.01) {
            robot.arm_motors(armPower);
            runtime.reset();
            while (runtime.milliseconds() < 100 && opModeIsActive()) {
            }
        }
        robot.arm_opening_system.setPower(0);
        telemetry.addData("status","robot has climbed down succesfully");
        telemetry.update();
        gyroTurn(0.5,179.9);

            if (getPosition() == GoldPosition.RIGHT) {
                gyroDrive(0.4, 5, 0);
                gyroTurn(0.2, 53);
                