package VectorFields;
import robocode.*;
//import java.awt.Color;

// API help : http://robocode.sourceforge.net/docs/robocode/robocode/Robot.html

/**
 * Obstacle - a robot by (your name here)
 */
public class Obstacle extends AdvancedRobot
{
	/**
	 * run: Obstacle's default behavior
	 */
	public void run() {
        setAdjustRadarForGunTurn(true);
		while(true) {
			turnGunRight(360);
		}
	}
}

