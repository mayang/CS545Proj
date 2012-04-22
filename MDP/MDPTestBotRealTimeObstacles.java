package mdp;

import robocode.Robot;
import robocode.ScannedRobotEvent;
import robocode.HitByBulletEvent;
import java.awt.Graphics2D;
import java.util.HashMap;
import java.util.Map;

import mdp.MDPUtility;

/*
 * Abstract: Robocode Robot that discretizes the battlefield into a grid of states and follows an MDP policy produced by value iteration
 * Date: 16 April 2012
 * Notes: This was the first MDP robot we created. It allowed us to test that our value iteration function, state discretization, action model, transition
 * model, and reward model functioned properly and had the desired effects. This robot works only in static environments.
 */
public class MDPTestBotRealTimeObstacles extends Robot {
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
	private String enemy_name = "mdp.MDPEnemyBotPerimeter*";
	//Policy
    private double[][][] transitions;
    private double[][] rewards;
    private double[][] q_table;
    private int[] policy = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    public boolean currently_updating;
    private int goal_state;
    long time1;
    long time2;
    double avg_velocity = 2.0;
    double current_velocity;
    double velocity_sum = 0.0;
    int random_walk = 10;
    int random_trigger_value = 10;
    int num_velocities = 0;
    long est_update_time = 40;
    double distance_trigger = 100.0;
    int last_valid_goal;
    int previous_goal;
    boolean ejected_policy;
    boolean first_update;
    volatile Map<String, Integer> obstacle_map = new HashMap<String,Integer>();
    ScannedRobotEvent sre;
    public void run() {
		currently_updating = false;
		ejected_policy = false;
		first_update = false;
    	 while (true) {
    		//Each time we get a turn, we find out the state we are in and execute the action our policy tells us to
         	int state = MDPUtility.getStateForXandY(getX(), getY());
        	if (getTime() % 10 == 0) {
        		turnRadarLeft(360);
        		continue;
        	}
         	if (policy[state] == MDPUtility.ACTION_NORTH) {
         		goNorth(50);
         	} else if (policy[state] == MDPUtility.ACTION_SOUTH) {
         		goSouth(50);
         	} else if (policy[state] == MDPUtility.ACTION_EAST) {
         		goEast(50);
         	} else if (policy[state] == MDPUtility.ACTION_WEST) {
         		goWest(50);
         	} else if (policy[state] == MDPUtility.ACTION_NORTHWEST) {
         		goNorthwest(50);
         	} else if (policy[state] == MDPUtility.ACTION_NORTHEAST) {
         		goNortheast(50);
         	} else if (policy[state] == MDPUtility.ACTION_SOUTHWEST) {
         		goSouthwest(50);
         	} else if (policy[state] == MDPUtility.ACTION_SOUTHEAST) {
         		goSoutheast(50);
         	} else if (policy[state] == -1) {
             	double r = Math.random();
             	if (r < 0.125) {
             		goNorth(30);
             	} else if (r >= 0.125 && r < 0.25) {
             		goSouth(30);
             	} else if (r >= 0.25 && r < 0.375) {
             		goEast(30);
             	} else if (r >= 0.375 && r < 0.5) {
             		goWest(30);
             	} else if (r >= 0.5 && r < 0.625) {
             		goNorthwest(30);
             	} else if (r >= 0.625 && r < 0.750) {
             		goSouthwest(30);
             	} else if (r >=0.75 && r < 0.875) {
             		goSoutheast(30);
             	} else if (r >= 0.875 && r < 1.0) {
             		goNortheast(30);
             	}
         	}
         	
        }
}

    /*
     * Fire when we scan a robot. Our policy keeps us oriented toward the enemy almost all the time, so firing straight ahead works, so
     * long as we keep our gun heading the same as our body heading. http://old.nabble.com/Using-Random-Statements-td4010734.html
     */
    public void onScannedRobot(ScannedRobotEvent e) {
    	double enemyBearing = getHeading() + e.getBearing(); 
    	double enemyX = getX() + e.getDistance() * Math.sin(Math.toRadians(enemyBearing)); 
    	double enemyY = getY() + e.getDistance() * Math.cos(Math.toRadians(enemyBearing));
    	sre = e;
    	if(!e.getName().equals(enemy_name)) {
        	synchronized(obstacle_map) {
        		obstacle_map.put(e.getName(), MDPUtility.getStateForXandY(enemyX, enemyY));
        	}
    	} else if (e.getName().equals(enemy_name)){
        	goal_state = MDPUtility.getStateForXandY(enemyX, enemyY);
        	previous_goal = goal_state;
    	if (!currently_updating) {
    		first_update = true;
    		time1 = getTime();
    		currently_updating = true;
    		Thread policy_update = new Thread() {
    			public void run() {
    					if (transitions == null || ejected_policy || sre.getDistance() > 150.0) {
    						System.out.print("Updating full\n");
    						transitions = MDPUtility.getTransitions();
    						rewards = MDPUtility.getRewardsWithObstacles(transitions, goal_state, obstacle_map);
    						q_table = MDPUtility.valueIteration(transitions, rewards);
        					policy = MDPUtility.generatePolicyFromQTable(q_table);
        					ejected_policy = false;
    					} else {
    						System.out.print("Updating partial \n");
    						rewards = MDPUtility.updateRewardsRealTimeWithObstacles(goal_state, previous_goal, transitions, rewards, obstacle_map);
    						q_table = MDPUtility.valueIterationRealTimeWithObstacles(goal_state, previous_goal, transitions, rewards, q_table, obstacle_map);
    						policy =  MDPUtility.updatePolicyRealTimeWithObstacles(policy, q_table, goal_state, previous_goal, obstacle_map);
    					}
    					doneUpdating();
    				}
				};
			policy_update.start();
			
    	}
        fire(1);
    	}
	}

	public void doneUpdating() {
		currently_updating = false;
		time2 = getTime();
		est_update_time = time2-time1;
		System.out.print(est_update_time + "\n");
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

