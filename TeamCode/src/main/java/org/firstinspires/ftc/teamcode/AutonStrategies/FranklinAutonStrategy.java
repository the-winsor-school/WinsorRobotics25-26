package org.firstinspires.ftc.teamcode.AutonStrategies;

import org.firstinspires.ftc.teamcode.Extensions.ThreadExtensions;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.FranklinRobot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Robot;
import org.firstinspires.ftc.teamcode.RobotModel.Robots.Wildbots2025;

public class FranklinAutonStrategy {
    /*
    the line below i'm not sure about... help
    (the public static one, shouldn't use wildbots2025 but...
    idk what the autonomousmecanumrobot is)
     */
    public static IAutonStrategy BasicAutonomous(FranklinRobot.AutonomousFranklinRobot robot) {
        return () -> {
            //insert code
        };
    }
    public static IAutonStrategy RedGoalAutonomous (FranklinRobot.AutonomousFranklinRobot robot) {
        return () -> {
            //Example: Navigate to red goal AprilTag (you'll need the actual tag ID)
            int redGoalTagId = 24;

            //Try to locate and drive to the red goal
            boolean foundGoal = false;
            for (int attempts = 0; attempts < 10 && !foundGoal; attempts++) {
                foundGoal = robot.driveToRedGoal (redGoalTagId, 12.0); //Get within 12 inches
                ThreadExtensions.TrySleep(500); //Wait between attempts
            }

            if (foundGoal) {
                //Successfully reached goal, now shoot!
                robot.mechAssembly.AutonShooter.StartShoot();
                ThreadExtensions.TrySleep(1000);

                robot.mechAssembly.FlappyServo.FlappyDown(); //Release balls
                ThreadExtensions.TrySleep(2000);

                robot.mechAssembly.FlappyServo.FlappyUp();
                robot.mechAssembly.AutonShooter.StopShoot();

            }
        };
    }
}
