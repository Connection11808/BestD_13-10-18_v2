package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static java.lang.Math.abs;

@Autonomous(name="functions", group="connection")
@Disabled
public class functions extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_Connection robot = new Hardware_Connection();
    AutoMainDetroit auto = new AutoMainDetroit();


    static final double COUNTS_PER_MOTOR_NEVEREST40 = 1120;    // eg: NEVEREST40 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION_NEVEREST40 = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CM = 10.5360;     // For figuring circumference
    static final double PULLEY_DIAMETER_CM = 4;
    static final double STEER = 0.93; //friction coefficiant.
    static final int COUNTS_PER_CM_ANDYMARK_WHEEL = (int) ((COUNTS_PER_MOTOR_NEVEREST40 * DRIVE_GEAR_REDUCTION_NEVEREST40) / (WHEEL_DIAMETER_CM * Pi.getNumber()) * STEER);

    static final double COUNTS_PER_MOTOR_TETRIX = 1440;
    static final int ARM_GEAR_REDUCTION_TETRIX = 9;

    private ElapsedTime runtime = new ElapsedTime();
    double DriveY = 0;
    double DriveX = 0;

    double TurnPower = 0;


    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.05;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.05;     // Larger is more responsive, but also less stable

    public boolean ErrorOnArmOpen=false;

    public enum gyroDriveDirection {
        LEFTandRIGHT,
        FORWARDandBACKWARD,
        DIAGONALRIGHT,
        DIAGONALLEFT
    }


    public enum motorType {
        ARM,
        DRIVE,
        OPENING_SYSTEM,
        All
    }

    private enum GoldPos {
        Right,
        Left,
        Center,
        None;
    }


    public GoldPos goldPos = GoldPos.None;

    public void runOpMode() {
     }


    public void gyroDrive(double speed,
                          int distance,
                          double angle,
                          gyroDriveDirection direction) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        int newRightBackTarget;


        robot.drivingSetMode(RUN_USING_ENCODER);
        double max;
        double error;
        double steer;
        double leftSpeed;
        double rightSpeed;
        double backSpeed;
        double frontSpeed;
        angle += gyroGetAngle();


        if (direction == gyroDriveDirection.FORWARDandBACKWARD) {
            telemetry.addData("gyroDrive", "gyroDrive");
            telemetry.update();

            // Ensure that the opmode is still active
            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                distance = (int) (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
                newLeftFrontTarget = robot.left_front_motor.getCurrentPosition() + distance;
                newRightFrontTarget = robot.right_front_motor.getCurrentPosition() + distance;
                newRightBackTarget = robot.right_back_motor.getCurrentPosition() + distance;
                newLeftBackTarget = robot.left_back_motor.getCurrentPosition() + distance;

                // Set Target and Turn On RUN_TO_POSITION
                robot.left_front_motor.setTargetPosition(newLeftFrontTarget);
                robot.right_front_motor.setTargetPosition(newRightFrontTarget);
                robot.right_back_motor.setTargetPosition(newRightBackTarget);
                robot.left_back_motor.setTargetPosition(newLeftBackTarget);

                robot.drivingSetMode(RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(abs(speed), 0.0, 1.0);
                robot.fullDriving(speed, speed);

                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive()) {
                    if (!robot.left_back_motor.isBusy() || !robot.left_front_motor.isBusy() ||
                            !robot.right_back_motor.isBusy() || !robot.right_front_motor.isBusy()) {
                        robot.fullDriving(0, 0);
                        break;
                    }

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;


                    robot.fullDriving(leftSpeed,rightSpeed);
                    // Display drive status for the driver.
                    telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Target", "%7d:%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("Actual", "%7d:%7d", robot.left_front_motor.getCurrentPosition(), robot.right_front_motor.getCurrentPosition(), robot.right_back_motor.getCurrentPosition(), robot.left_back_motor.getCurrentPosition());
                    telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                    telemetry.update();
                }


                // Stop all motion;
                robot.fullDriving(0, 0);
                robot.drivingSetMode(RUN_USING_ENCODER);
            }

        } else if (direction == gyroDriveDirection.LEFTandRIGHT) {

            if (opModeIsActive()) {

                // Determine new target position, and pass to motor controller
                distance = (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
                newLeftFrontTarget = robot.left_front_motor.getCurrentPosition() - distance;
                newRightFrontTarget = robot.right_front_motor.getCurrentPosition() + distance;
                newRightBackTarget = robot.right_back_motor.getCurrentPosition() - distance;
                newLeftBackTarget = robot.left_back_motor.getCurrentPosition() + distance;

                // Set Target and Turn On RUN_TO_POSITION
                robot.left_front_motor.setTargetPosition(newLeftFrontTarget);
                robot.right_front_motor.setTargetPosition(newRightFrontTarget);
                robot.right_back_motor.setTargetPosition(newRightBackTarget);
                robot.left_back_motor.setTargetPosition(newLeftBackTarget);

                robot.left_back_motor.setMode(RUN_TO_POSITION);
                robot.left_front_motor.setMode(RUN_TO_POSITION);
                robot.right_back_motor.setMode(RUN_TO_POSITION);
                robot.right_front_motor.setMode(RUN_TO_POSITION);

                // start motion.
                speed = Range.clip(abs(speed), 0.0, 1.0);
                robot.driveToLEFTandRIGHT(speed, speed);


                // keep looping while we are still active, and BOTH motors are running.
                while (opModeIsActive() && (robot.left_back_motor.isBusy() && robot.left_front_motor.isBusy() &&
                        robot.right_back_motor.isBusy() && robot.right_front_motor.isBusy())) {

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0) {
                        steer *= -1.0;
                    }
                    frontSpeed = speed + steer;
                    backSpeed = speed - steer;

                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(abs(backSpeed), abs(frontSpeed));
                    if (max > 1.0) {
                        backSpeed /= max;
                        frontSpeed /= max;
                    }


                    robot.driveToLEFTandRIGHT(backSpeed, frontSpeed);


                    // Display drive status for the driver.
                    telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Target", "%7d:%7d", newLeftFrontTarget, newRightFrontTarget, newLeftBackTarget, newRightBackTarget);
                    telemetry.addData("Actual", "%7d:%7d", robot.left_front_motor.getCurrentPosition(), robot.right_front_motor.getCurrentPosition(), robot.left_back_motor.getCurrentPosition(), robot.right_back_motor.getCurrentPosition());
                    telemetry.update();
                }


                // Stop all motion;
                robot.fullDriving(0, 0);

            }
        } else if (direction == gyroDriveDirection.DIAGONALLEFT) {
            distance = (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
            newRightFrontTarget = robot.right_front_motor.getCurrentPosition() + distance;
            newLeftBackTarget = robot.left_back_motor.getCurrentPosition() + distance;

            // Set Target and Turn On RUN_TO_POSITION
            robot.right_front_motor.setTargetPosition(newRightFrontTarget);
            robot.left_back_motor.setTargetPosition(newLeftBackTarget);

            robot.left_back_motor.setMode(RUN_TO_POSITION);
            robot.right_front_motor.setMode(RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(abs(speed), 0.0, 1.0);
            robot.driveToLEFTandRIGHT(speed, speed);

            if (opModeIsActive()) {
                distance = (int) (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
                if (distance < 0) {
                    speed = -speed;
                }
                while (opModeIsActive() && robot.left_front_motor.isBusy() && robot.left_back_motor.isBusy()){

                    // adjust relative speed based on heading error.
                    error = getError(angle);
                    steer = getSteer(error, P_DRIVE_COEFF);

                    // if driving in reverse, the motor correction also needs to be reversed
                    if (distance < 0)
                        steer *= -1.0;

                    leftSpeed = speed - steer;
                    rightSpeed = speed + steer;

                    // Normalize speeds if either one exceeds +/- 1.0;
                    max = Math.max(abs(leftSpeed), abs(rightSpeed));
                    if (max > 1.0) {
                        leftSpeed /= max;
                        rightSpeed /= max;
                    }

                    robot.left_back_motor.setPower(leftSpeed);
                    robot.right_front_motor.setPower(-rightSpeed);

                    // Display drive status for the driver.
                    telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                    telemetry.addData("Target", "%7d:%7d", newRightFrontTarget, newLeftBackTarget);
                    telemetry.addData("Actual", "%7d:%7d", robot.right_front_motor.getCurrentPosition(), robot.left_back_motor.getCurrentPosition());
                    telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                    telemetry.update();
                }

            }
                // Stop all motion;
                robot.fullDriving(0, 0);
                robot.drivingSetMode(RUN_USING_ENCODER);
        }
        else if (direction == gyroDriveDirection.DIAGONALRIGHT) {
            distance = (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
            newRightFrontTarget = robot.right_front_motor.getCurrentPosition() + distance;
            newLeftBackTarget = robot.left_back_motor.getCurrentPosition() + distance;

            // Set Target and Turn On RUN_TO_POSITION
            robot.right_front_motor.setTargetPosition(newRightFrontTarget);
            robot.left_back_motor.setTargetPosition(newLeftBackTarget);

            robot.left_back_motor.setMode(RUN_TO_POSITION);
            robot.right_front_motor.setMode(RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(abs(speed), 0.0, 1.0);
            robot.driveToLEFTandRIGHT(speed, speed);

            if (opModeIsActive()) {
                distance = (int) (distance * COUNTS_PER_CM_ANDYMARK_WHEEL);
                if (distance < 0) {
                    speed = -speed;
                }
            }
            while (opModeIsActive() && robot.left_front_motor.isBusy() && robot.right_back_motor.isBusy()) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(abs(leftSpeed), abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.left_front_motor.setPower(leftSpeed);
                robot.right_back_motor.setPower(-rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St", "%5.1f/%5.1f", error, steer);
                telemetry.addData("Target", "%7d:%7d", newRightFrontTarget, newLeftBackTarget);
                telemetry.addData("Actual", "%7d:%7d", robot.right_front_motor.getCurrentPosition(), robot.left_back_motor.getCurrentPosition());
                telemetry.addData("Speed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
                telemetry.update();
            }
            // Stop all motion;
            robot.fullDriving(0, 0);
            robot.drivingSetMode(RUN_USING_ENCODER);
        }
    }
    public void gyroTurn(double speed, double angle) {
        angle += gyroGetAngle();

        robot.drivingSetMode(RUN_USING_ENCODER);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
        }
    }


    boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double leftSpeed;
        double rightSpeed;


        // determine turn power based on +/- error
        error = getError(angle);

        if (abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
            telemetry.addData("status", "on target");
            telemetry.update();
        } else {
            steer = getSteer(error, PCoeff);
            rightSpeed = speed * steer;
            leftSpeed = -rightSpeed;
            telemetry.addData("Target", "%5.2f", angle);
            telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
            telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);
            telemetry.update();
        }

        // Send desired speeds to motors.
        robot.fullDriving(leftSpeed, rightSpeed);

        // Display it for the driver.

        return onTarget;
    }


    public double getError(double targetAngle) {

        double robotError;
        // calculate error in -179 to +180 range  (
        double angle = gyroGetAngle();
        robotError = targetAngle - angle;

        while (robotError > 180 && opModeIsActive()) {
            robotError -= 360;
        }
        while (robotError <= -180 && opModeIsActive()) {
            robotError += 360;
        }
        return robotError;
    }


    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    public GoldPos GetGoldMineralPosition(){
        return GoldPos.None;
    }




    public void climbDown(){
        robot.team_marker_servo.setPosition(0);
        robot.arm_motors(-0.5);
        armOpeningEncoder(1, 10);
        robot.arm_motors(0);
        robot.team_marker_servo.setPosition(0);
        runtime.reset();
        while(runtime.milliseconds() < 1000 && opModeIsActive()){
            robot.arm_motors(1);
        }
        robot.arm_motors(0);
        gyroDrive(1, -5, 0, gyroDriveDirection.FORWARDandBACKWARD);
        gyroDrive(1, -5, 0, gyroDriveDirection.LEFTandRIGHT);
    }


    public void mineralToLanderCrater() {
        robot.arm_motors(0.5);
        gyroDrive(1,-70,15, gyroDriveDirection.LEFTandRIGHT);
        armEncoder(1,3000);
        robot.arm_motors(0.25);
        armOpeningEncoder(1,45);
        robot.mineral_keeper_servo.setPosition(1);
        armEncoder(1,-3000);
        
    }

    public float gyroGetAngle() {
        //telemetry.addData("y: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).firstAngle);
        telemetry.addData("z: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).secondAngle);
        //telemetry.addData("x: ", robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).thirdAngle);
        return robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    public void putTeamMarker() {
        gyroDrive(1,15,0, gyroDriveDirection.FORWARDandBACKWARD);
        gyroDrive(1,115,-45, gyroDriveDirection.LEFTandRIGHT);
        gyroTurn(1, -20);
        pickUpMineral();
        gyroDrive(1,-130,-10, gyroDriveDirection.FORWARDandBACKWARD);
        robot.team_marker_servo.setPosition(0);
        gyroDrive(1,100,0, gyroDriveDirection.FORWARDandBACKWARD);
        robot.team_marker_servo.setPosition(1);
        gyroDrive(1,-28,0, gyroDriveDirection.LEFTandRIGHT);
        gyroTurn(1, 45);
        }

    public void pickUpMineral(){
        robot.arm_motors(-0.15);
        armOpeningEncoder(1,30);
        robot.arm_motors(0);
        robot.MineralIn();
        armOpeningEncoder(1,10);
        robot.StopMineral();
        robot.arm_motors(0.15);
        armOpeningEncoder(1,-30);
        robot.arm_motors(0);
    }

    public void sampling(){
        gyroDrive(1,10,0, gyroDriveDirection.FORWARDandBACKWARD);
        switch (goldPos){
            case Left:
                gyroTurn(0.8,30);
                break;
            case Right:
                gyroTurn(0.8,-30);
                break;
        }
        robot.arm_motors(-0.15);
        armOpeningEncoder(1,30);
        armOpeningEncoder(1,-30);
        robot.arm_motors(0);

    }
    public void armOpeningEncoder(double speed, double distance) {
        robot.arm_opening_system.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm_opening_system.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.arm_opening_system_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm_opening_system_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int StartPos=robot.arm_opening_system.getCurrentPosition();


        int Target = (int) (distance * (COUNTS_PER_MOTOR_NEVEREST40 / (Pi.getNumber() * PULLEY_DIAMETER_CM)));
        Target += robot.arm_opening_system.getCurrentPosition();
        Target *= -1;

        robot.arm_opening_system.setTargetPosition(Target);
        robot.arm_opening_system.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        while (ErrorOnMotor(robot.arm_opening_system, StartPos) &&
                abs(Target) > abs(robot.arm_opening_system.getCurrentPosition()) &&
                abs(Target) > abs(robot.arm_opening_system_2.getCurrentPosition()) &&
                opModeIsActive()) {
            telemetry.addData("current:", robot.arm_opening_system.getCurrentPosition());
            telemetry.addData("current_2:", robot.arm_opening_system_2.getCurrentPosition());
            telemetry.addData("target:", Target);
            telemetry.update();
            if (Target < robot.arm_opening_system.getCurrentPosition()) {
                robot.arm_opening_system.setPower(speed);
                robot.arm_opening_system_2.setPower(speed);
            }
            if (Target > robot.arm_opening_system.getCurrentPosition()) {
                robot.arm_opening_system.setPower(-speed);
                robot.arm_opening_system_2.setPower(-speed);
            }
        }
        robot.arm_opening_system.setPower(0);
    }

    public void armEncoder(double speed, double Target) {
        robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.arm_motor_2.setTargetPosition((int)Target);
        telemetry.addData("target", Target);
        telemetry.addData("current", robot.arm_motor_2.getCurrentPosition());
        telemetry.update();
        int StartPos = robot.arm_motor_2.getCurrentPosition();
        while (abs(robot.arm_motor_2.getCurrentPosition()) < abs((int) Target) && ErrorOnMotor(robot.arm_motor_2,StartPos) && opModeIsActive()) {
            telemetry.addData("current:", robot.arm_motor_2.getCurrentPosition());
            telemetry.addData("target:", Target);
            telemetry.update();
            robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (Target > robot.arm_motor_2.getCurrentPosition()) {
                robot.arm_motors(speed);
            }
            if (Target < robot.arm_motor_2.getCurrentPosition()) {
                robot.arm_motors(-speed);
            }
            robot.arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("current1:", robot.arm_motor_2.getCurrentPosition());
            telemetry.addData("target1:", Target);
            telemetry.update();
        }
        robot.arm_motors(0);
    }

    public void samplingDepot(){
        robot.team_marker_servo.setPosition(0);
        robot.drivingSetMode(RUN_WITHOUT_ENCODER);
        runtime.reset();
        gyroDrive(0.4, -10, 0, gyroDriveDirection.FORWARDandBACKWARD);

        switch(goldPos){
            case Right:
                gyroDrive(1, 70, 0, gyroDriveDirection.DIAGONALRIGHT);
                gyroDrive(1, -70, 0, gyroDriveDirection.DIAGONALRIGHT);
                telemetry.addData("Status", "going to right");
            case Left:
                gyroDrive(1, 70, 0, gyroDriveDirection.DIAGONALLEFT);
                gyroDrive(1, -70, 0, gyroDriveDirection.DIAGONALLEFT);
                telemetry.addData("Status", "going to left");
            case Center:
                gyroDrive(1 , 20, 0, gyroDriveDirection.FORWARDandBACKWARD);
                gyroDrive(1 , -20, 0, gyroDriveDirection.FORWARDandBACKWARD);
                telemetry.addData("Status", "going to Center");
        }

    }

    public void putTeamMarkerDepot() {
        gyroDrive(1,30,0,gyroDriveDirection.FORWARDandBACKWARD);
        robot.arm_motors(-0.1);
        armOpeningEncoder(1,3);
        robot.arm_motors(0);
        armOpeningEncoder(1,47);
        //mineral output
        robot.arm_motors(0.45);
        armOpeningEncoder(1,-50);
        robot.arm_motors(0);
    }

    public void goToCraterDepot() {
        gyroDrive(11,-100,0,gyroDriveDirection.LEFTandRIGHT);
        gyroTurn(1,45);
        gyroDrive(1,-50,0,gyroDriveDirection.FORWARDandBACKWARD);

    }

    public void InitRobot(){
        robot.init(hardwareMap);

        robot.team_marker_servo.setPosition(0.7);
        //waiting for the user to press start.
        robot.fullEncoderSetMode(RUN_WITHOUT_ENCODER);
        telemetry.addData("angle", gyroGetAngle());
        telemetry.update();
        waitForStart();
    }
    public boolean ErrorOnMotor(DcMotor motor,int StartPos){
        boolean ErrorMotor = false;
        if(runtime.seconds()>0.5 && robot.IntInRange(StartPos-30,StartPos+30,motor.getCurrentPosition())){
            ErrorMotor=true;
        }
        return ErrorMotor;
    }
}