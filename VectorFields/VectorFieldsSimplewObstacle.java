package VectorFields;

import robocode.*;
import java.lang.*;

/**
 * Simple Robocode robot that uses vector fields to navigate towards a goal. 
 */
public class VectorFieldsSimplewObstacle extends Robot
{
    private static final int SCREEN_WIDTH = 600;
    private static final int SCREEN_HEIGHT = 600;
    private static final double MAX_DISTANCE = Math.sqrt(Math.pow(SCREEN_WIDTH, 2) + Math.pow(SCREEN_HEIGHT, 2));
    private static final int MAX_SPEED = 10;
    private static final int GOAL_DISTANCE = 200;

    private boolean foundGoal = false;
	private boolean foundObstacle = false;
    private double goalX, goalY;
	private double obstacleX, obstacleY;
    /**
     * If the robot doesn't know where it's goal is at, it tries to locate it by spinning in a circle.
     * Otherwise, it attempts to orient it self to the goal.
     */
    public void run()
    {
        double robotX, robotY, heading;
        double angleToGoal;
		double angleToObstacle;
		double goalTurn = 0;
		double obstacleTurn = 0;
		double turn;
		double effect;
		
        while (true)
        {
			if (foundGoal || foundObstacle) {
				robotX = getX();
				robotY = getY();
				// Robocode headings return 0 degrees as North, 90 as East, etc
				heading = 360 - (getHeading() - 90);
				if (foundGoal)
				{
					angleToGoal = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));

					if (angleToGoal < 0)
					{
						angleToGoal += 360;
					}

					//turnLeft(angleToGoal - heading);
					goalTurn = angleToGoal - heading;
					
					//ahead(Math.min(10, distance));
					//ahead(calcRobotSpeedLinear(robotX, robotY, goalX, goalY));
					//ahead(calcRobotSpeedGlobalFields(robotX, robotY, goalX, goalY));
				}
				if (foundObstacle) {
					angleToObstacle = Math.toDegrees(Math.atan2(obstacleY - robotY, obstacleX- robotX));
					double dist = Math.sqrt(Math.pow(obstacleX - robotX, 2.0) + Math.pow(obstacleY - robotY, 2.0));
					effect = (100.0 - dist) / 100.0;
					//System.out.println("effect" + effect);
					if (effect >= 0) {
					//if (dist < 100) {
						if (angleToObstacle < 0) {
							angleToObstacle += 360;
						}
						obstacleTurn = angleToObstacle - heading;
						obstacleTurn *= effect;
						obstacleTurn *= -1;
					} 
				}
				turn = goalTurn + obstacleTurn;
				turnLeft(turn);

				//ahead(Math.min(10, distance));
				ahead(calcRobotSpeedLinear(robotX, robotY, goalX, goalY));
			}
            else
            {
                turnRight(1);
				ahead(1);
            }
        }
    }

    /**
     * On Scanning a robot, set it as our goal.
     */
    public void onScannedRobot(ScannedRobotEvent e)
    {
		String eName = e.getName();
		String roboName = eName.substring(eName.indexOf(".")+1, eName.length()-1);
		System.out.println("I see " + roboName);
		System.out.println("It is " + e.getDistance() + " away!");
		double enemyBearing = getHeading() + e.getBearing();
		double enemyX = getX() + e.getDistance() * Math.sin(Math.toRadians(enemyBearing));
		double enemyY = getY() + e.getDistance() * Math.cos(Math.toRadians(enemyBearing));
		if (roboName.startsWith("Goal")) {
			foundGoal = true;
			goalX = enemyX;
			goalY = enemyY;
		}
		else if (roboName.startsWith("Obstacle")) {
			//double dist = Math.sqrt(Math.pow(obstacleX - robotX, 2.0) + Math.pow(obstacleY - robotY, 2.0));

			//if (dist < 50) {
				foundObstacle = true;
				obstacleX = enemyX;
				obstacleY = enemyY;
			//}
		
		}
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
	public void onHitWall(HitWallEvent e) {
		turnLeft(180);
		ahead(1);
	}
}
