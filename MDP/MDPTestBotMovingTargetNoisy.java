package mdp;

import robocode.Robot;
import robocode.ScannedRobotEvent;
import robocode.HitByBulletEvent;
import java.awt.Graphics2D;
import mdp.MDPUtility;

/*
 * Abstract: Robocode robot that scans for its target (the only other robot on the battlefield). Once it has found the enemy,
 * it determines the (x,y) position of the enemy and the state associated with that enemy position. It then uses this "goal"
 * state to update its reward function and runs value iteration to find an optimal policy to lead itself to the "goal". The enemy
 * robot is stationary in this case. It is only "moving" in the sense that, between rounds, it will change position. Unlike our
 * MDPTestBot, this bot can account for this change in position, update its rewards, run value iteration online, and converge to 
 * any stationary goal state, rather than just a single stationary goal state. This robot has noise in its motions
 * Author: TJ Collins
 * Notes: This robot was the second step in our exploration of MDPs in Robocode. It was the logical next step after MDPTestBot 
 * because it required us to integrate value iteration into the Robocode runtime and to use our sensors in order to determine
 * where the "goal" was.
 */
public class MDPTestBotMovingTargetNoisy extends Robot {
	//Constants for orientation
	private final double NORTH = 0.0;
	private final double NORTH_ALT = 360.0;
	private final double SOUTH = 180.0;
	private final double EAST = 90.0;
	private final double WEST = 270.0;
	private final double NORTHWEST = 315.0;
	private final double NORTHEAST = 45.0;
	private final double SOUTHWEST = 225.0;
	private final double SOUTHEAST = 135.0;
	//Policy
    private double[][][] transitions;
    private double[][] rewards;
    private double[][] q_table;
    private int[] policy = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    public boolean currently_updating;
    private int goal_state;
    public void run() {
    	this.setAllColors(java.awt.Color.white);
		currently_updating = false;
    	 while (true) {
        	//Each time we get a turn, we find out the state we are in and execute the action our policy tells us to
        	int state = MDPUtility.getStateForXandY(getX(), getY());
        	//Add noise into motion with 0.2 probability. As the robot is heading "straight", it veers to the right or left by 1 state
        	//with a 20% chance.
        	double random = Math.random();
        	if (random > 0.8) System.out.print("Oops...slip\n");
        	if (policy[state] == MDPUtility.ACTION_NORTH) {
        		if (random <=0.8) goNorth(10);
        		else if (random > 0.8 && random < 0.9) goNortheast(10);
        		else goNorthwest(10);
        	} else if (policy[state] == MDPUtility.ACTION_SOUTH) {
        		if (random <=0.8) goSouth(10);
        		else if (random > 0.8 && random < 0.9) goSoutheast(10);
        		else goSouthwest(10);
        	} else if (policy[state] == MDPUtility.ACTION_EAST) {
        		if (random <=0.8) goEast(10);
        		else if (random > 0.8 && random < 0.9) goNortheast(10);
        		else goSoutheast(10);
        	} else if (policy[state] == MDPUtility.ACTION_WEST) {
        		if (random <=0.8) goWest(10);
        		else if (random > 0.8 && random < 0.9) goNorthwest(10);
        		else goSouthwest(10);
        	} else if (policy[state] == MDPUtility.ACTION_NORTHWEST) {
        		if (random <=0.8) goNorthwest(10);
        		else if (random > 0.8 && random < 0.9) goNorth(10);
        		else goWest(10);
        	} else if (policy[state] == MDPUtility.ACTION_NORTHEAST) {
        		if (random <=0.8) goNortheast(10);
        		else if (random > 0.8 && random < 0.9) goNorth(10);
        		else goEast(10);
        	} else if (policy[state] == MDPUtility.ACTION_SOUTHWEST) {
        		if (random <=0.8) goSouthwest(10);
        		else if (random > 0.8 && random < 0.9) goSouth(10);
        		else goWest(10);
        	} else if (policy[state] == MDPUtility.ACTION_SOUTHEAST) {
        		if (random <=0.8) goSoutheast(10);
        		else if (random > 0.8 && random < 0.9) goSouth(10);
        		else goEast(10);
        	} else if (policy[state] == -1) {
            	turnRadarLeft(360);
        	}
        }
}

    /*
     * Fire when we scan a robot. Our policy keeps us oriented toward the enemy almost all the time, so firing straight ahead works, so
     * long as we keep our gun heading the same as our body heading. 
     */
    public void onScannedRobot(ScannedRobotEvent e) {
    	double enemyBearing = getHeading() + e.getBearing(); 
    	//Calculate enemy (x,y) position
    	double enemyX = getX() + e.getDistance() * Math.sin(Math.toRadians(enemyBearing)); 
    	double enemyY = getY() + e.getDistance() * Math.cos(Math.toRadians(enemyBearing));
    	//Find new goal state for the enemy (x,y) position
    	goal_state = MDPUtility.getStateForXandY(enemyX, enemyY);
    	//If we aren't currently updating the policy
    	if (!currently_updating) {
    		currently_updating = true;
    		//Spawn a new thread because we can't block the main run thread for as long as we need to do value iteration
    		Thread policy_update = new Thread() {
    			public void run() {
    					//Update the transitions once
    					if (transitions == null) transitions = MDPUtility.getTransitionsNoisy();
    					//Get the rewards for the new goal
    					rewards = MDPUtility.getRewards(transitions, goal_state);
    					//Generate Q-table with value iteration
    					q_table = MDPUtility.valueIteration(transitions, rewards);
    					//Generate new policy from the Q-table
    					policy = MDPUtility.generatePolicyFromQTable(q_table);
    					//Let main thread know that we are done updating
    					doneUpdating();
			    	}
				};
				//Start the policy update thread
			policy_update.start();
    	}
        fire(1);
	}

	public void doneUpdating() {
		//Allow for more policy updates
		currently_updating = false;
	}
	
	public void onHitByBullet(HitByBulletEvent e) {
        
	}

	/*
	 * This just prints out some information on the Robocode battlefield itself. Useful for debugging purposes.
	 * Shows you the state, heading, gun heading, energy, radar heading, x position, and y position of the MDP 
	 * robot. This is how we verified our motion model and tiling function primarily.
	 */
	public void onPaint(Graphics2D g) {
    	// Set the paint color to red
    	g.setColor(java.awt.Color.WHITE);
    	// Paint a filled rectangle at (50,50) at size 100x150 pixels
    	int state = MDPUtility.getStateForXandY(getX(), getY());
    	String state_str = Integer.toString(state);
    	String xpos_str = Double.toString(getX());
    	String ypos_str = Double.toString(getY());
    	String heading = Double.toString(getHeading());
    	String gunheading = Double.toString(getGunHeading());
    	String energy = Double.toString(getEnergy());
    	String radarheading = Double.toString(getRadarHeading());
    	energy = "Energy: " + energy;
    	xpos_str = "X: " + xpos_str;
    	ypos_str = "Y: " + ypos_str;
    	heading = "Heading: " + heading;
    	gunheading = "Gun Heading: " + gunheading;
    	radarheading = "Radar Heading: " + radarheading;
    	state_str = "State: " + state_str;
    	char[] xpos = xpos_str.toCharArray();
    	char[] ypos = ypos_str.toCharArray();
    	char[] headingchars = heading.toCharArray();
    	char[] gunheadingchars = gunheading.toCharArray();
    	char[] energychars = energy.toCharArray();
    	char[] radarheadingchars = radarheading.toCharArray();
    	char[] statechars = state_str.toCharArray();
    	g.drawChars(statechars, 0, statechars.length, 250, 125);
    	g.drawChars(radarheadingchars, 0, radarheadingchars.length, 250, 110);
    	g.drawChars(energychars, 0, energychars.length, 250, 95);
    	g.drawChars(gunheadingchars, 0, gunheadingchars.length, 250, 80);
    	g.drawChars(headingchars, 0, headingchars.length, 250, 65);
    	g.drawChars(xpos, 0, xpos.length, 250, 50);
    	g.drawChars(ypos, 0, ypos.length, 250, 35);
	}
	
	//Action to turn and head north from any initial orientation
	public void goNorth(int distance) {
		double current_heading = getHeading();
		if (current_heading <= SOUTH) {
			turnLeft(current_heading - NORTH);
		} else {
			turnRight(NORTH_ALT - current_heading);
		}
		ahead(distance);
	}
	
	//Action to turn and head south from any initial orientation
	public void goSouth(int distance) {
		double current_heading = getHeading();
		if (current_heading <= SOUTH) {
			turnRight(SOUTH - current_heading);
		} else {
			turnLeft(current_heading - SOUTH);
		}
		ahead(distance);
	}
	
	//Action to turn and head east from any initial orientation
	public void goEast(int distance) {
		double current_heading = getHeading();
		if (current_heading >= WEST || current_heading < EAST) {
			if (current_heading < 360.0 && current_heading >= WEST) turnRight(NORTH_ALT - current_heading + EAST);
			else turnRight(EAST - current_heading);
		} else {
			turnLeft(current_heading - EAST);
		}
		ahead(distance);
	}
	
	//Action to turn and head west from any initial orientation
	public void goWest(int distance) {
		double current_heading = getHeading();
		if (current_heading > WEST || current_heading <= EAST) {
			if (current_heading < 360.0 && current_heading >= WEST) turnLeft(current_heading - WEST);
			else turnLeft(NORTH_ALT - WEST + current_heading);
		} else {
			turnRight(WEST - current_heading);
		}
		ahead(distance);
	}
	
	//Action to turn and head northwest from any initial orientation
	public void goNorthwest(int distance) {
		double current_heading = getHeading();
		if (current_heading > NORTHWEST || current_heading <= SOUTHEAST) {
			if (current_heading < 360.0 && current_heading >= NORTHWEST) turnLeft(current_heading - NORTHWEST);
			else turnLeft(NORTH_ALT - NORTHWEST + current_heading);
		} else {
			turnRight(NORTHWEST - current_heading);
		}
		ahead(distance);
	}
	
	//Action to turn and head northwest from any initial orientation
	public void goSouthwest(int distance) {
		double current_heading = getHeading();
		if (current_heading >= NORTHEAST && current_heading < SOUTHWEST) {
			turnRight(SOUTHWEST - current_heading);
		} else {
			if (current_heading < 360.0 && current_heading >= SOUTHWEST) turnLeft(current_heading - SOUTHWEST);
			else turnLeft(current_heading + NORTH_ALT - SOUTHWEST);
		}
		ahead(distance);
	}
	
	//Action to turn and head northeast from any initial orientation
	public void goNortheast(int distance) {
		double current_heading = getHeading();
		if (current_heading >= NORTHEAST && current_heading < SOUTHWEST) {
			turnLeft(current_heading - NORTHEAST);
		} else {
			if (current_heading < 360.0 && current_heading >= SOUTHWEST) turnRight(NORTH_ALT - current_heading + NORTHEAST);
			else turnRight(NORTHEAST - current_heading);
		}
		ahead(distance);
	}
	
	//Action to turn and head southeast from any initial orientation
	public void goSoutheast(int distance) {
		double current_heading = getHeading();
		if (current_heading > NORTHWEST || current_heading < SOUTHEAST) {
			if (current_heading < 360.0 && current_heading >= NORTHWEST) turnRight(NORTH_ALT - current_heading + SOUTHEAST);
			else turnRight(SOUTHEAST - current_heading);
		} else {
			turnLeft(current_heading - SOUTHEAST);
		}
		ahead(distance);
	}
}

