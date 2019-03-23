package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous(name="Depot", group="Auto")

public class AutoDepotDetroit extends functions {

    //the main function.
    public void runOpMode() {

        //function that initialize the robot.
        InitRobot();

        //checks if the opMode is active.
        if(opModeIsActive()) {
            //active the Depot autonomous.
            auto.Detroit(false);
        }
    }

}