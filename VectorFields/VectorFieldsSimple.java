package VectorFields;

import robocode.*;
import java.lang.*;

/**
 * Simple Robocode robot that uses vector fields to navigate towards a goal. 
 */
public class VectorFieldsSimple extends Robot
{
    private static final int SCREEN_WIDTH = 600;
    private static final int SCREEN_HEIGHT = 600;
    private static final double MAX_DISTANCE = Math.sqrt(Math.pow(SCREEN_WIDTH, 2) + Math.pow(SCREEN_HEIGHT, 2));
    private static final int MAX_SPEED = 10;
    private static final int GOAL_DISTANCE = 200;

    private boolean foundGoal = false;
    private double goalX, goalY;

    /**
     * If the robot doesn't know where it's goal is at, it tries to locate it by spinning in a circle.
     * Otherwise, it attempts to orient it self to the goal.
     */
    public void run()
    {
        double robotX, robotY, heading;
        double angleToGoal;

        while (true)
        {
            if (foundGoal)
            {

                robotX = getX();
                robotY = getY();
                // Robocode headings return 0 degrees as North, 90 as East, etc
                heading = 360 - (getHeading() - 90);
                angleToGoal = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));

                if (angleToGoal < 0)
                {
                    angleToGoal += 360;
                }

                turnLeft(angleToGoal - heading);

                //ahead(Math.min(10, distance));
                ahead(calcRobotSpeedLinear(robotX, robotY, goalX, goalY));
                //ahead(calcRobotSpeedGlobalFields(robotX, robotY, goalX, goalY));
            }
            else
            {
                turnRight(1);
            }
        }
    }

    /**
     * On Scanning a robot, set it as our goal.
     */
    public void onScannedRobot(ScannedRobotEvent e)
    {
        double enemyBearing = getHeading() + e.getBearing();
        double enemyX = getX() + e.getDistance() * Math.sin(Math.toRadians(enemyBearing));
        double enemyY = getY() + e.getDistance() * Math.cos(Math.toRadians(enemyBearing));

        foundGoal = true;
        goalX = enemyX;
        goalY = enemyY;
        System.out.println("Enemy is at " + enemyX + " " + enemyY);
    }

    /**
     * Linearly decay the speed of the robot when it nears the goal.
     */
    public int calcRobotSpeedLinear(double robotX, double robotY, double goalX, double goalY)
    {
        int speed = 0;

        double distance = Math.sqrt(Math.pow(robotX - goalX, 2) + Math.pow(robotY - goalY, 2));
        if (distance >= GOAL_DISTANCE)
        {
            speed = MAX_SPEED;
        }
        else
        {
            speed = (int)((distance / GOAL_DISTANCE) * MAX_SPEED + 0.5);
        }
        
        return speed;
    }

    /**
     * Decay the speed "globally." The goal doens't have a range at which it starts to cause a speed drop off.
     * Instead, the entire screen has a scaled gradient.
     */
    public int calcRobotSpeedGlobalFields(double robotX, double robotY, double goalX, double goalY)
    {
        double distance = Math.sqrt(Math.pow(robotX - goalX, 2) + Math.pow(robotY - goalY, 2));
        return (int)((distance / MAX_DISTANCE) * MAX_SPEED + 0.5);
    }
}
