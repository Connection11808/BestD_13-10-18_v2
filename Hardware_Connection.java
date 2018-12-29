package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static java.lang.Math.abs;
import static java.lang.Math.atan;

//En Connection

public class Hardware_Connection {
    public ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double DRIVE_SPEED = 0.7;     // Nominal speed for better accuracy.
    static final double TURN_SPEED = 0.5;     // Nominal half speed for better accuracy.

    /* Public OpMode members. */
    public DcMotor right_front_motor = null;
    public DcMotor right_back_motor = null;
    public DcMotor left_front_motor = null;
    public DcMotor left_back_motor = null;
    public DcMotor arm_motor_1 = null;
    public DcMotor arm_motor_2 = null;
    public DcMotor arm_opening_system = null;
    public DcMotor arm_collecting_system = null;
    public BNO055IMU gyro = null;
    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    /* Constructor */
    public Hardware_Connection() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map


        hwMap = ahwMap;

        // Define and Initialize Motors
        left_back_motor = hwMap.get(DcMotor.class, "LB");
        right_back_motor = hwMap.get(DcMotor.class, "RB");
        left_front_motor = hwMap.get(DcMotor.class, "LF");
        right_front_motor = hwMap.get(DcMotor.class, "RF");
        arm_motor_1 = hwMap.get(DcMotor.class, "ARM1");
        arm_motor_2 = hwMap.get(DcMotor.class, "ARM2");
        arm_opening_system = hwMap.get(DcMotor.class, "AOS");
        arm_collecting_system = hwMap.get(DcMotor.class, "ACS");
        gyro = hwMap.get(BNO055IMU.class, "imu");


        left_front_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        right_front_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back_motor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power

        fullReset();

        right_back_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gyro = hwMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        fullDriving(0, 0);
        arm_motor_1.setPower(0);
        arm_motor_2.setPower(0);
        arm_opening_system.setPower(0);
        arm_collecting_system.setPower(0);
    }

    public void arm_motors(double power) {
        arm_motor_1.setPower(power);
        arm_motor_2.setPower(power);
    }


    //function that makes you able to control all the driving motors at once
    public void fullDriving(double LeftPower, double RightPower) {
        left_back_motor.setPower(LeftPower);
        left_front_motor.setPower(LeftPower);
        right_back_motor.setPower(RightPower);
        right_front_motor.setPower(RightPower);
    }

    public void rightDriveY(double power) {
        right_back_motor.setPower(-power);
        right_front_motor.setPower(power);
    }

    public void leftDriveY(double leftPower) {
        left_back_motor.setPower(-leftPower);
        left_front_motor.setPower(-leftPower);
    }


    //function that makes you able to control the robot to drive left and right
    public void leftDriveX(double leftPower) {
        left_back_motor.setPower(leftPower);
        left_front_motor.setPower(-leftPower);
    }

    public void rightDriveX(double rightPower) {
        right_back_motor.setPower(rightPower);
        right_front_motor.setPower(-rightPower);
    }


    public void allMotors(double power) {
        fullDriving(power, power);
        arm_collecting_system.setPower(power);
        arm_opening_system.setPower(power);
        arm_motor_1.setPower(power);
        arm_motor_2.setPower(power);
    }

    public void fullReset() {
        fullDriving(0, 0);
        arm_collecting_system.setPower(0);
        arm_opening_system.setPower(0);
        arm_motor_1.setPower(0);
        arm_motor_2.setPower(0);
    }

    public void fullEncoder() {
        left_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_motor_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_motor_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_opening_system.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_collecting_system.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void fullEncoderReset() {
        left_back_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_opening_system.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_collecting_system.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void driveToLeft(double leftPower, double maxSpeed) {
        left_back_motor.setPower(leftPower * maxSpeed);
        left_front_motor.setPower(-leftPower * maxSpeed);
        right_front_motor.setPower(leftPower * maxSpeed);
        right_back_motor.setPower(-leftPower * maxSpeed);
    }

    public void driveToRight(double rightPower, double maxSpeed) {
        left_back_motor.setPower(-rightPower * maxSpeed);
        left_front_motor.setPower(rightPower * maxSpeed);
        right_front_motor.setPower(-rightPower * maxSpeed);
        right_back_motor.setPower(rightPower * maxSpeed);
    }
}


    /*public double whichQuarter(double x, double y, double unusedZone) {
        double angale;

        if (x > 0) {
            angale = atan(y / x) * 180. / 3.14159;
            return angale;
        } else {
            return 0;
        }
    }*/

    /*public int whichQuarter(double x, double y, double unusedZone) {
        if(abs(x) < unusedZone && abs(y) < unusedZone) {
            return 0;
        }
        if(y > abs(x)){
            return 1;
        }
        else if(x > abs(y)){
            return 2 ;
        }
        else if(y < -abs(x)){
            return 3;
        }
        else{
            return 4;
        }
    }*/

