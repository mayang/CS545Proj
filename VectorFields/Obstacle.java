package VectorFields;
import robocode.*;
//import java.awt.Color;

// API help : http://robocode.sourceforge.net/docs/robocode/robocode/Robot.html

/**
 * Obstacle - a robot by (your name here)
 */
public class Obstacle extends Robot
{
	/**
	 * run: Obstacle's default behavior
	 */
	public void run() {
		// Initialization of the robot should be put here

		// After trying out your robot, try uncommenting the import at the top,
		// and the next line:

		// setColors(Color.red,Color.blue,Color.green); // body,gun,radar

		// Robot main loop
		while(true) {
			turnGunRight(360);
			//ahead(580);
            //turnLeft(90);
		}
	}
	}

