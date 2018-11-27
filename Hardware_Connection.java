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

//En Connection

public class Hardware_Connection {
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    /* Public OpMode members. */
    public DcMotor right_front_motor = null;
    public DcMotor right_back_motor = null;
    public DcMotor left_front_motor = null;
    public DcMotor left_back_motor = null;
    public DcMotor arm_motor_1 = null;
    public DcMotor arm_motor_2 = null;
    public DcMotor arm_opening_system = null;
    public BNO055IMU gyro= null;
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
        left_back_motor = hwMap.get(DcMotor.class, "left back");
        right_back_motor = hwMap.get(DcMotor.class, "right back");
        left_front_motor = hwMap.get(DcMotor.class, "left front");
        right_front_motor = hwMap.get(DcMotor.class, "right front");
        arm_motor_1 = hwMap.get(DcMotor.class, "arm motor 1");
        arm_motor_2 = hwMap.get(DcMotor.class, "arm motor 2");
        arm_opening_system = hwMap.get(DcMotor.class, "arm opening system");


        left_front_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        left_back_motor.setPower(0);

        left_front_motor.setPower(0);
        left_back_motor.setPower(0);
        right_back_motor.setPower(0);
        right_front_motor.setPower(0);
        arm_motor_1.setPower(0);
        arm_motor_2.setPower(0);
        arm_opening_system.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_back_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gyro = hwMap.get(BNO055IMU.class, "imu");
        gyro.initialize(parameters);
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


    }
    public void arm_motors(double power){
        arm_motor_1.setPower(power);
        arm_motor_2.setPower(power);
    }
    public void arm_motors_REVERSE(double power){
        arm_motor_1.setPower(-power);
        arm_motor_2.setPower(-power);
}

}

