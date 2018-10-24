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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class Hardware_Connection_BestD {
    /* Public OpMode members. */
    public DcMotor right_front_motor = null;
    public DcMotor right_back_motor = null;
    public DcMotor left_front_motor = null;
    public DcMotor left_back_motor = null;
    public DcMotor left_claw_part = null;
    public DcMotor right_claw_part = null;
    public DcMotor claw_opening_system = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware_Connection_BestD() {

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
        left_claw_part = hwMap.get(DcMotor.class, "left claw motor");
        right_claw_part = hwMap.get(DcMotor.class, "right claw motor");
        claw_opening_system = hwMap.get(DcMotor.class, "claw opening system");


        // Set all motors to zero power
        left_back_motor.setPower(0);
        left_front_motor.setPower(0);
        right_back_motor.setPower(0);
        right_front_motor.setPower(0);
        right_claw_part.setPower(0);
        left_claw_part.setPower(0);
        claw_opening_system.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        left_back_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.
    }
    public void claw_motor(double power){
        left_claw_part.setPower(power);
        right_claw_part.setPower(power);
    }
}

