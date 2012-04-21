package VectorFields;

import robocode.*;
import java.lang.*;

/**
 * Robocode robot that locates and navigates toward a goal while also avoiding obstacles that it gets near.
 */
public class VectorFieldsObstacleAvoid extends AdvancedRobot
{
    private static final int SCREEN_WIDTH = 600;
    private static final int SCREEN_HEIGHT = 600;
    private static final double MAX_DISTANCE = Math.sqrt(Math.pow(SCREEN_WIDTH, 2) + Math.pow(SCREEN_HEIGHT, 2));
    private static final int MAX_SPEED = 10;
    private static final int GOAL_DISTANCE = 50;
    private static final int OBJ_DISTANCE = 150;

    private double goalX, goalY;
    private double obsX, obsY;
    private boolean foundGoal = false;
    private boolean foundObstacle = false;

    public void run()
    {
        setTurnRadarRight(Double.POSITIVE_INFINITY);

        double robotX, robotY;
        double robotHeading, angleToGoal, angleToObj;
        double adjustment;
        double obsAngle, obsAdjustment;
        double angleDiff;

        double speedToGoal, speedFromObj;

        while (true)
        {
            if (foundGoal)
            {
                robotX = getX();
                robotY = getY();

                robotHeading = 360 - (getHeading() - 90);
                angleToGoal = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
                if (angleToGoal < 0)
                {
                    angleToGoal += 360;
                }

                adjustment = angleToGoal - robotHeading;
                adjustment = normalizeAngle(adjustment);
                speedToGoal = calcRobotSpeedLinear(robotX, robotY, goalX, goalY);

                if (foundObstacle)
                {
                    speedFromObj = calcObjRepulseSpeed(robotX, robotY, obsX, obsY);

                    if (speedFromObj != 0)
                    {
                        obsAngle = Math.toDegrees(Math.atan2(robotY - obsY, robotX - obsX));
                        if (obsAngle < 0)
                            obsAngle += 360;

                        angleDiff = obsAngle - angleToGoal;
                        angleDiff = normalizeAngle(angleDiff);
                        adjustment += (angleDiff * (speedFromObj / speedToGoal));
                        speedToGoal -= speedFromObj;
                    }
                }

                adjustment = normalizeAngle(adjustment);
                setTurnLeft(adjustment);
                setAhead(speedToGoal);
            }

            execute();
        }
    }

    public void onScannedRobot(ScannedRobotEvent e)
    {
        double targetBearing = getHeading() + e.getBearing();
        double tmpX = getX() + e.getDistance() * Math.sin(Math.toRadians(targetBearing));
        double tmpY = getY() + e.getDistance() * Math.cos(Math.toRadians(targetBearing));

        if (e.getName().equals("VectorFields.VectorFieldsGoal*") && !foundGoal)
        {
            foundGoal = true;
            goalX = tmpX;
            goalY = tmpY;
        }
        else if (e.getName().equals("sample.SittingDuck") && !foundObstacle)
        {
            System.out.println("Obstacle at " + tmpX + " " + tmpY);
            foundObstacle = true;
            obsX = tmpX;
            obsY = tmpY;
        }

        setTurnRadarRight(getRadarTurnRemaining());
    }

    /**
     * Linearly decay the speed of the robot when it nears the goal.
     */
    public double calcRobotSpeedLinear(double robotX, double robotY, double goalX, double goalY)
    {
        double speed = 0;

        double distance = Math.sqrt(Math.pow(robotX - goalX, 2) + Math.pow(robotY - goalY, 2));
        if (distance >= GOAL_DISTANCE)
        {
            speed = MAX_SPEED;
        }
        else
        {
            speed = (distance / GOAL_DISTANCE) * MAX_SPEED + 0.5;
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

    public double normalizeAngle(double angle)
    {
        if (angle <= -360)
        {
            angle += 360;
        }
        else if (angle >= 360)
        {
            angle -= 360;
        }

        if (angle < -180)
        {
            angle += 360;
        }
        else if (angle > 180)
        {
            angle -= 360;
        }

        return angle;
    }

    public double calcObjRepulseSpeed(double robotX, double robotY, double obsX, double obsY)
    {
        double speed = 0.0;
        double distance = Math.sqrt(Math.pow(robotX - obsX, 2) + Math.pow(robotY - obsY, 2));

        if (distance <= OBJ_DISTANCE)
        {
            speed = ((OBJ_DISTANCE - distance) / OBJ_DISTANCE) * MAX_SPEED;
        }

        return speed;
    }
}
