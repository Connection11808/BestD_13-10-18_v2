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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="ConnectionTeleop", group="Pushbot")

public class ConnectionTeleop extends OpMode {

    /* Declare OpMode members. */
    Hardware_Connection robot = new Hardware_Connection();
    // could also use HardwarePushbotMatrix class.
    double left_speed;
    double right_speed;
    double arm_power;
    double arm_reverse_power;


    /**
     0* Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /** Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        robot.right_back_motor.setPower(0);
        robot.right_front_motor.setPower(0);
        robot.left_back_motor.setPower(0);
        robot.left_front_motor.setPower(0);


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left_speed = -gamepad1.left_stick_y;
        right_speed = gamepad1.right_stick_y;
        telemetry.addData("right drive value", right_speed);
        telemetry.addData("left drive value", left_speed);
        telemetry.update();


        robot.left_front_motor.setPower(left_speed);
        robot.left_back_motor.setPower(left_speed);
        robot.right_front_motor.setPower(right_speed);
        robot.right_back_motor.setPower(right_speed);

        arm_reverse_power = arm_power *-1;

        arm_power = gamepad2.right_trigger;
        arm_reverse_power = gamepad2.left_trigger;

        robot.arm_motor.setPower(arm_power);
        robot.arm_motor.setPower(arm_reverse_power*-1);

        if (arm_power>0.75) {
            arm_power =0.75;

        }
        if (arm_reverse_power>-0.75) {
            arm_reverse_power =-0.75;

        }


        if (gamepad2.right_bumper) {
            robot.arm_opening_system.setPower(1);
        }
        if (gamepad2.left_bumper) {
            robot.arm_opening_system.setPower(-1);
        }
        robot.arm_opening_system.setPower(0);
    }

    public void stop () {
        }

    }