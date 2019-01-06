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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ConnectionTp", group="Connection")

public class ConnectionTp extends LinearOpMode {
    public void runOpMode() {
        /* Declare OpMode members. */
        Hardware_Connection robot = new Hardware_Connection();
        double armPower;
        double maxSpeed = 1;
        int mode = 1;

        robot.init(hardwareMap);
        telemetry.addData(".", "done init ");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            /*if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_y != 0) {
                robot.fullDriving(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
            }*/
            telemetry.addData("rightJoystickY value", gamepad1.right_stick_y);
            telemetry.addData("leftJoystickY value", gamepad1.left_stick_y);
            telemetry.update();

            armPower = gamepad2.left_trigger - gamepad2.right_trigger;

            robot.arm_motor_2.setPower((armPower) * maxSpeed);
            if (armPower > 0.5) {
                armPower = 0.5;
            }
            if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
                robot.arm_motor_1.setPower(0);
            }


            /*if (gamepad2.x) {
                robot.arm_collecting_system.setPower(0.8);
            } else if (gamepad2.a) {
                robot.arm_collecting_system.setPower(-0.8);
            }


            if (gamepad2.y && maxSpeed == 0.6) {
                maxSpeed = 1;
                while (gamepad2.y) {
                }
            }
            if (gamepad2.y && maxSpeed == 0.8) {
                maxSpeed = 0.4;
                while (gamepad2.y) {
                }
            }

            if (!gamepad2.a && !gamepad2.x) {
                robot.arm_collecting_system.setPower(0);
            }*/
            if (gamepad2.left_bumper) {
                robot.arm_opening_system.setPower(1);
            } else if (gamepad2.right_bumper) {
                robot.arm_opening_system.setPower(-1);
            }
            if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
                robot.arm_opening_system.setPower(0);
            }

            /*if (gamepad1.x) {
                robot.driveToLeft(-0.5, 0.5);
            } else if (gamepad1.b) {
                robot.driveToRight(-0.5, 0.5);
            }
            /*if (gamepad1.right_stick_y == 0 && gamepad1.left_stick_y == 0 && !gamepad1.b && !gamepad1.x) {
                robot.fullDriving(0, 0);
            }

            if (gamepad1.right_trigger > 0) {
                robot.driveToRight(-gamepad1.right_trigger, 1);
            }
            else if (gamepad1.left_trigger > 0) {
                robot.driveToLeft(-gamepad1.left_trigger, 1);
            }
            //else if (gamepad1.left_trigger == gamepad1.right_trigger || (gamepad1.left_trigger > 0 && gamepad1.right_trigger > 0)) {
              //  robot.fullDriving(0, 0);
            }*/
            if(gamepad1.y){
                mode = 1;
            }
            if(gamepad1.b){
                mode = 2;
            }
            if(mode == 1) {
                int left_quarter_straight = robot.whichQuarterStraight(-gamepad1.left_stick_y, gamepad1.left_stick_x, 0.2);
                int right_quarter_straight = robot.whichQuarterStraight(-gamepad1.right_stick_y, gamepad1.right_stick_x, 0.2);

                switch (left_quarter_straight) {
                    case 0:
                        robot.leftDriveY(0, 0);
                        break;
                    case 1:
                        robot.leftDriveY(-gamepad1.left_stick_y, 1);
                        break;
                    case 2:
                        robot.leftDriveX(-gamepad1.left_stick_x, 1);
                        break;
                    case 3:
                        robot.leftDriveY(-gamepad1.left_stick_y, 1);
                        break;
                    case 4:
                        robot.leftDriveX(-gamepad1.left_stick_x, 1);
                        break;
                }

                switch (right_quarter_straight) {
                    case 0:
                        robot.rightDriveX(0, 0);
                        break;
                    case 1:
                        robot.rightDriveY(-gamepad1.right_stick_y, 1);
                        break;
                    case 2:
                        robot.rightDriveX(-gamepad1.right_stick_x, 1);
                        break;
                    case 3:
                        robot.rightDriveY(-gamepad1.right_stick_y, 1);
                        break;
                    case 4:
                        robot.rightDriveX(-gamepad1.right_stick_x, 1);
                        break;
                }
            }
            /*if(mode == 2){
                int right_quarter_diagonal = robot.whichQuarterDiagonal(gamepad1.right_stick_y, gamepad1.right_stick_x, 0.2);
                int left_Quarter_diagonal = robot.whichQuarterDiagonal(gamepad1.left_stick_y, gamepad1.left_stick_x, 0.2);

                if(right_quarter_diagonal == 0 && left_Quarter_diagonal == 0){
                    robot.fullDriving(0, 0);
                }
                if(right_quarter_diagonal == 1 && left_Quarter_diagonal == 1){
                    robot.diagonalDrive1(1, 1);
                }
                else if(right_quarter_diagonal == 2 && left_Quarter_diagonal == 2){
                    robot.diagonalDrive2(1, 1);
                }
                else if(right_quarter_diagonal == 3 && left_Quarter_diagonal == 3){
                    robot.diagonalDrive1(1, 1);
                }
                else{
                    robot.diagonalDrive2(1, 1);
                }
            }*/
        }
    }
}
