package VectorFields;

import robocode.*;
import java.lang.*;
import java.util.*;
import java.awt.Color;

/**
 * Robocode robot that locates and navigates toward a goal while also avoiding obstacles that it gets near.
 */
public class VectorFieldsMultMovObsAvoid extends AdvancedRobot
{
    private static final int SCREEN_WIDTH = 600;
    private static final int SCREEN_HEIGHT = 600;
    private static final double MAX_DISTANCE = Math.sqrt(Math.pow(SCREEN_WIDTH, 2) + Math.pow(SCREEN_HEIGHT, 2));
    private static final int MAX_SPEED = 5;
    private static final int GOAL_DISTANCE = 50;
    private static final int OBJ_DISTANCE = 200;
    private static final String GOAL_NAME = "VectorFields.VectorFieldsMovingGoal*";

    private double goalX, goalY;
    private double obsX, obsY;
    private boolean foundGoal = false;
    private boolean foundObstacle = false;

    private Map<String, Enemy> obstacles;

    public void run()
    {
        setAllColors(Color.GREEN);
        setTurnRadarRight(Double.POSITIVE_INFINITY);

        double robotX, robotY;
        double robotHeading, angleToGoal, angleToObs;
        double adjustment;
        double obsAngle, obsAdjustment;
        double angleDiff;
        double speedToGoal, speedFromObs;

        Enemy temp;
        obstacles = new HashMap<String, Enemy>(10);

        while (true)
        {
            if (foundGoal)
            {
                robotX = getX();
                robotY = getY();
                goalX = obstacles.get(GOAL_NAME).x;
                goalY = obstacles.get(GOAL_NAME).y;

                // Adjust robocode's returned heading so that 0 aligns with the positive x-axis instead of the positive y-axis.
                // Also make it so that positive angle indicates a counter clockwise rotation instead of the clockwise style
                // returned by robocode.
                robotHeading = 360 - (getHeading() - 90);
                angleToGoal = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
                if (angleToGoal < 0)
                {
                    angleToGoal += 360;
                }

                adjustment = angleToGoal - robotHeading;
                adjustment = normalizeAngle(adjustment);
                speedToGoal = calcRobotSpeedLinear(robotX, robotY, goalX, goalY);

                // Calculate how the robot's heading and speed should be affected by objects that it has located
                // as it explores the world.
                Iterator it = obstacles.entrySet().iterator();
                while (it.hasNext()) 
                {
                    System.out.println("Iterating through objects.");

                    Map.Entry pairs = (Map.Entry)it.next();

                    // If the current item in the Iterator isn't the goal.
                    if (!pairs.getKey().equals(GOAL_NAME))
                    {
                        temp = (Enemy)pairs.getValue();
                        speedFromObs = calcObjRepulseSpeed(robotX, robotY, temp.x, temp.y);

                        // If the robot is in range of the object to be affected by it's repulsion.
                        if (speedFromObs != 0)
                        {
                            obsAngle = Math.toDegrees(Math.atan2(robotY - temp.y, robotX - temp.x));
                            if (obsAngle < 0)
                                obsAngle += 360;

                            angleDiff = obsAngle - angleToGoal;
                            angleDiff = normalizeAngle(angleDiff);
                            adjustment += (angleDiff * (speedFromObs / speedToGoal));
                            speedToGoal -= speedFromObs;
                        }
                    }

                    // Was getting a null pointer exception when using this. The internet lied about its usefulness.
                    //it.remove(); // avoids a ConcurrentModificationException
                }

                adjustment = normalizeAngle(adjustment);
                setTurnLeft(adjustment);
                //ahead(speedToGoal);
                setAhead(speedToGoal);
            }

            execute();
        }
    }

    /**
     * When scanning a robot we need to add it to the collection of scanned objects so it can
     * be used later for updates to the bots movement.
     */
    public void onScannedRobot(ScannedRobotEvent e)
    {
        double targetBearing = getHeading() + e.getBearing();
        double tmpX = getX() + e.getDistance() * Math.sin(Math.toRadians(targetBearing));
        double tmpY = getY() + e.getDistance() * Math.cos(Math.toRadians(targetBearing));
        String name = e.getName();

        if (name.equals(GOAL_NAME))
        {
            foundGoal = true;
        }

        obstacles.put(name, new Enemy(tmpX, tmpY, e.getBearing()));

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

    /**
     * Normalize an angle so it falls in the range -180 to 180 in the least efficient manner possible.
     */
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

    /**
     * Calculate how repusled the scanning robot is by a nearby object.
     */
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
