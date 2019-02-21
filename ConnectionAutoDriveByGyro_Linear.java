/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static java.lang.Math.abs;

@Autonomous(name="ConnectionGyroTest", group="Auto")

public class ConnectionAutoDriveByGyro_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware_Connection robot = new Hardware_Connection();   // Use a Pushbot's hardware
    ConnectionAutoDepot autoFunctions = new ConnectionAutoDepot();

    static final double COUNTS_PER_MOTOR_NEVEREST40 = 1120;    // eg: NEVEREST40 Motor Encoder
    static final double DRIVE_GEAR_REDUCTION_NEVEREST40 = 1;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_CM = 10.5360;     // For figuring circumference
    static final double PULLEY_DIAMETER_CM = 4;
    static final double STEER = 0.93; //friction coefficiant
    static final int COUNTS_PER_CM_ANDYMARK_WHEEL = (int) ((COUNTS_PER_MOTOR_NEVEREST40 * DRIVE_GEAR_REDUCTION_NEVEREST40) / (WHEEL_DIAMETER_CM * Pi.getNumber()) * STEER);
    static final int COUNTS_PER_CM_ANDYMARK_PULLEY = (int) ((COUNTS_PER_MOTOR_NEVEREST40 * DRIVE_GEAR_REDUCTION_NEVEREST40) / (PULLEY_DIAMETER_CM * Pi.getNumber()));
    static final double HEADING_THRESHOLD = 1;      // As tight as we can make it with an integer gyro
    static final double P_TURN_COEFF = 0.05;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_COEFF = 0.05;     // Larger is more responsive, but also less stable
    static final int COUNTS_PER_CM_OPENING = (int) ((COUNTS_PER_MOTOR_NEVEREST40 * DRIVE_GEAR_REDUCTION_NEVEREST40) / PULLEY_DIAMETER_CM * Pi.getNumber());

    static final double COUNTS_PER_MOTOR_TETRIX = 1440;
    static final int ARM_GEAR_REDUCTION_TETRIX = 1 / 9;
    static final int TETRIX_MOTOR_ANGLES = (int) (COUNTS_PER_MOTOR_TETRIX * (float) ARM_GEAR_REDUCTION_TETRIX / 360);


    @Override
    public void runOpMode() {

        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
        robot.init(hardwareMap);

        robot.encoderSetMode(RUN_USING_ENCODER);
        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.left_back_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.left_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_back_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_front_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Send telemetry message to alert driver that we are calibrating;
        telemetry.addData(">", "Calibrating Gyro");    //
        telemetry.update();


        // make sure the gyro is calibrated before continuing


        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();


        robot.left_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.left_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_front_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.right_back_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (Display Gyro value), and reset gyro before we move..
        while (!isStarted()) {
            telemetry.addData(">", "Robot Heading = %d", autoFunctions.gyroGetAngle());
            telemetry.update();
        }


        autoFunctions.gyroDrive(0.6, 40, 0, ConnectionAutoDepot.gyroDriveDirection.FORWARDandBACKWARD);
        autoFunctions.gyroDrive(0.4, -40, 0, ConnectionAutoDepot.gyroDriveDirection.LEFTandRIGHT);
        autoFunctions.gyroDrive(0.6, -40, 0, ConnectionAutoDepot.gyroDriveDirection.FORWARDandBACKWARD);
        autoFunctions.gyroDrive(0.4, 40, 0, ConnectionAutoDepot.gyroDriveDirection.LEFTandRIGHT);

    }
}