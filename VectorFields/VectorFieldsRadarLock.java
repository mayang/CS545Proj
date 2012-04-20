package VectorFields;

import robocode.*;
import java.lang.*;

/**
 * A Robocode robot that attempts to use vector fields to navigate towards a moving goal. This
 * uses an advanced robot in an effort to smooth out the motion issues with the simple robots.
 */
public class VectorFieldsRadarLock extends AdvancedRobot
{
    private static final int SCREEN_WIDTH = 600;
    private static final int SCREEN_HEIGHT = 600;
    private static final double MAX_DISTANCE = Math.sqrt(Math.pow(SCREEN_WIDTH, 2) + Math.pow(SCREEN_HEIGHT, 2));
    private static final int MAX_SPEED = 20;
    private static final int GOAL_DISTANCE = 50;

    private boolean foundGoal = false;
    private double goalX, goalY;

    public void run()
    {
        double angleToGoal, adjustment;
        double robotX, robotY, heading;

        setAdjustRadarForRobotTurn(true);

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

                adjustment = angleToGoal - heading;

                if (adjustment >= 360)
                {
                    adjustment -= 360;
                }
                else if (adjustment <= -360)
                {
                    adjustment += 360;
                }

                if (adjustment < -180)
                {
                    adjustment += 360;
                }
                else if (adjustment > 180)
                {
                    adjustment -= 360;
                }

                setTurnLeft(adjustment);
                ahead(calcRobotSpeedLinear(robotX, robotY, goalX, goalY));
            }
            else
            {
                setTurnRadarRight(360);
            }

            scan();
            //execute();
        }
    }

    /** 
     * On scanning any other robot, set it as the current goal.
     */
    public void onScannedRobot(ScannedRobotEvent e)
    {
        double enemyBearing = getHeading() + e.getBearing();
        double enemyX = getX() + e.getDistance() * Math.sin(Math.toRadians(enemyBearing));
        double enemyY = getY() + e.getDistance() * Math.cos(Math.toRadians(enemyBearing));

        foundGoal = true;
        goalX = enemyX;
        goalY = enemyY;

        double radarTurn = getHeadingRadians() + e.getBearingRadians() - getRadarHeadingRadians();
        setTurnRadarRightRadians(robocode.util.Utils.normalRelativeAngle(radarTurn));
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
