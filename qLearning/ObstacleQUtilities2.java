 package qLearning;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class ObstacleQUtilities2 {
	public static final int ROW_LENGTH = 30;
	public static final double TILE_SIZE = 600/ROW_LENGTH;
	public static final int NUM_STATES = ROW_LENGTH * ROW_LENGTH;
	public static final int NUM_ACTIONS = 8;
	public static final int NUM_EPISODES = 100000;
	public static final int GOAL_STATE = 329;
	public static final int[] OBSTACLES = {200,15,200,45,200,75,200,105,200,135,200,165,200,195,200,230,200,265,200,300};
	public static final Set<Integer> OBSTACLE_STATES = obstacleXYsToStates(OBSTACLES);
	public static final double alpha = 0.5;
	public static final double gamma = 0.9;
	public static final double epsilon = 0.3;

	public static final int ACTION_NORTH = 0;
	public static final int ACTION_SOUTH = 1;
	public static final int ACTION_EAST = 2;
	public static final int ACTION_WEST = 3;
	public static final int ACTION_NORTHEAST = 4;
	public static final int ACTION_NORTHWEST = 5;
	public static final int ACTION_SOUTHEAST = 6;
	public static final int ACTION_SOUTHWEST = 7;
	
	public static int[] QtoPolicy(double[][] QTable) {
		int[] policy = new int[NUM_STATES];
		for(int i=0; i< ObstacleQUtilities2.NUM_STATES; i++) {
			policy[i] = maxA(i, QTable);
				}
		return policy;
	}
	
	public static double[][] generateQTable(int goal) {
		return generateQTable(goal, -1);
	}
	
	public static double[][] generateQTable(int goal, int current) {
		double[][] Q = new double[ObstacleQUtilities2.NUM_STATES][ObstacleQUtilities2.NUM_ACTIONS];
		for(int episode = 0; episode < NUM_EPISODES; episode++) {
			int T = 11;
			int s;
			if (current == -1) {
				s = (int)Math.random() * ObstacleQUtilities2.NUM_STATES;
			} else {
				s = current;
			}
			int sNext;
//			int a = epsilonPickAction(s, Q, 0.0);
			int a = noisyBoltzmannPickAction(s, Q, T);	
			double r = -1;
			double rNext;
//			int step = 1;
			
			while (true) {
				if(isTerminal(s, goal)) {
					break;
				}
//				step += 1;
				if (T <= 1) {
					T = 1;
				} else {
					T = T/2;
				}
				sNext = transitionResult(s, a);
//				rNext = transitionReward(s, a, sNext);
				rNext = heuristicTransitionReward(s, sNext, goal);
				
				updateQ(Q, s, a, r, sNext);
				s = sNext;
				r = rNext;
//				a = epsilonPickAction(s, Q, QUtilities.epsilon);
				a = noisyBoltzmannPickAction(s, Q, T);
			}
		}
		
		return Q;
		
	}
	
	public static Set<Integer> obstacleXYsToStates(int[] obstacleXYs) {
		Set<Integer> result = new HashSet<Integer>(obstacleXYs.length/2);
		for(int i = 0; i < obstacleXYs.length; i += 2) {
			result.add(XYtoState(obstacleXYs[i], obstacleXYs[i+ 1]));
		}
		return result;
	}
	
	public static void updateQ(double[][] QTable, int curState, int curAction, double r, int nextState){
		double curQ = QTable[curState][curAction];
		QTable[curState][curAction] = curQ + (ObstacleQUtilities2.alpha * (r + ObstacleQUtilities2.gamma * (maxAQ(nextState, QTable)- curQ)));
	}
	
	public static int maxA(int state, double[][] Q) {
		int maxA = -1;
		double curMax = -1000000; 
		for(int j=0; j<ObstacleQUtilities2.NUM_ACTIONS; j++) {
			if (Q[state][j] > curMax) {
				maxA = j;
				curMax = Q[state][j];
			}
		}
		return maxA;
	}
	
	public static double maxAQ(int nextState, double[][] QTable){
		double maxQ = -1000;
		for (int i=0; i < ObstacleQUtilities2.NUM_ACTIONS; i++) {
			if (QTable[nextState][i] > maxQ) {
				maxQ = QTable[nextState][i];
			}
		}
		return maxQ;
	}
	
	public static double heuristicTransitionReward(int state, int sNext, int goal) {
		// If it hits a wall -100, reaches goal +100, else -1
		if(state == goal) {
			return 100;
		// This assumes the current transition model where ending up in the same state means you hit a wall. As I extend that out, we'll need to adapt this.
		} else if (state == sNext) {
			return -200;
		} else {
			return - Math.sqrt(Math.pow((double) stateToX(goal) - stateToX(state), 2.0) + Math.pow((stateToY(goal) - stateToY(state)), 2.0));
		}
	}
	
	public static int transitionResult(int state, int action){
		int trialNextState = -1;
		switch(action){
		case ObstacleQUtilities2.ACTION_NORTH:
			if (! nextToTopWall(state)) {
				trialNextState = state + ObstacleQUtilities2.ROW_LENGTH;
				break;
			}else {
				return state;
			}
		case ObstacleQUtilities2.ACTION_NORTHEAST:
			if (! nextToTopWall(state) && ! nextToRightWall(state)) {
				trialNextState = state + ObstacleQUtilities2.ROW_LENGTH + 1;
				break;
			} else {
				return state;
			}
		case ObstacleQUtilities2.ACTION_EAST:
			if (! nextToRightWall(state)) {
				trialNextState = state + 1;
				break;
			} else {
				return state;
			}
		case ObstacleQUtilities2.ACTION_SOUTHEAST:
			if (! nextToBottomWall(state) && ! nextToRightWall(state)) {
				trialNextState = state - ObstacleQUtilities2.ROW_LENGTH + 1;
				break;
			} else {
				return state;
			}
		case ObstacleQUtilities2.ACTION_SOUTH:
			if (! nextToBottomWall(state)) {
				trialNextState = state - ObstacleQUtilities2.ROW_LENGTH;
				break;
			} else {
				return state;
			}
		case ObstacleQUtilities2.ACTION_SOUTHWEST:
			if (! nextToLeftWall(state) && ! nextToBottomWall(state)) {
				trialNextState = state - ObstacleQUtilities2.ROW_LENGTH -1;
				break;
			} else {
				return state;
			}
		case ObstacleQUtilities2.ACTION_WEST:
			if (! nextToLeftWall(state)) {
				trialNextState = state -1;
				break;
			} else {
				return state;
			}
		case ObstacleQUtilities2.ACTION_NORTHWEST:
			if (! nextToLeftWall(state) && ! nextToTopWall(state)) {
				trialNextState = state + ObstacleQUtilities2.ROW_LENGTH -1;
				break;
			} else {
				return state;
			}
		}
		if(OBSTACLE_STATES.contains(trialNextState)) {
			return state;
		} else {
			return trialNextState;
		}
	}
	
	public static boolean nextToTopWall(int state) {
		if ((state/ROW_LENGTH) >= (ROW_LENGTH -1)){
			return true;
		} else {
			return false;
		}
	}
	
	public static boolean nextToBottomWall(int state) {
		if ((state/ROW_LENGTH) <= 0){
			return true;
		} else {
			return false;
		}
	}
	
	public static boolean nextToRightWall(int state) {
		if ((state % ROW_LENGTH) >= (ROW_LENGTH -1)){
			return true;
		} else {
			return false;
		}
	}
	
	public static boolean nextToLeftWall(int state) {
		if ((state % ROW_LENGTH) <= 0){
			return true;
		} else {
			return false;
		}
	}
	
	public static int stateToX(int state) {
		return state % ROW_LENGTH;
	}
	
	public static int stateToY(int state) {
		return state / ROW_LENGTH;
	}
	
	public static boolean isTerminal (int state, int goal) {
		if (state == goal) {
			return true;
		} else {
			return false;
		}
	}
	
	public static int epsilonPickAction(int s2, double[][] Q, double epsilon) {
		if(Math.random() > epsilon) {
			return maxA(s2, Q);
		} else {
			return (int) Math.random() * 8;
		}
	}
	
	public static int noisyBoltzmannPickAction(int s2, double[][] Q, double T) {
		/*
		 * Part of the simplified noisy model. Each move has an 80% chance of going in 
		 * the chosen direction and a 10% chance of going in each of the neighboring 
		 * two direction (So, a move N would have 10% chance of going NE/NW).
		 */
		int ideal = boltzmannPickAction(s2, Q, T);
		double rand = Math.random();
		switch (ideal){
		case 0:
			if(rand < 0.8) {
				return 0;
			} else if (rand < 0.9) {
				return 7;
			} else {
				return 1;
			}
		case 1:
			if(rand < 0.8) {
				return 1;
			} else if (rand < 0.9) {
				return 2;
			} else {
				return 0;
			}
		case 2:
			if(rand < 0.8) {
				return 2;
			} else if (rand < 0.9) {
				return 1;
			} else {
				return 3;
			}
		case 3:
			if(rand < 0.8) {
				return 3;
			} else if (rand < 0.9) {
				return 2;
			} else {
				return 4;
			}
		case 4:
			if(rand < 0.8) {
				return 4;
			} else if (rand < 0.9) {
				return 3;
			} else {
				return 5;
			}
		case 5:
			if(rand < 0.8) {
				return 5;
			} else if (rand < 0.9) {
				return 4;
			} else {
				return 6;
			}
		case 6:
			if(rand < 0.8) {
				return 6;
			} else if (rand < 0.9) {
				return 5;
			} else {
				return 7;
			}
		case 7:
			if(rand < 0.8) {
				return 7;
			} else if (rand < 0.9) {
				return 6;
			} else {
				return 0;
			}
		}
		return 0;
	}

	public static int boltzmannPickAction(int s2, double[][] Q, double T) {
		double r = Math.random();
		
		double val0 = Math.exp(Q[s2][0]/T);
		double val1 = Math.exp(Q[s2][1]/T);
		double val2 = Math.exp(Q[s2][2]/T);
		double val3 = Math.exp(Q[s2][3]/T);
		double val4 = Math.exp(Q[s2][4]/T);
		double val5 = Math.exp(Q[s2][5]/T);
		double val6 = Math.exp(Q[s2][6]/T);
		double val7 = Math.exp(Q[s2][7]/T);
		
		double total = val0 + val1 + val2 + val3 + val4 + val5 + val6 + val7;
		
		List<Double> probabilities = new ArrayList<Double>(8);
		
		probabilities.add(val0 /total);
		probabilities.add(val1 /total);
		probabilities.add(val2 /total);
		probabilities.add(val3 /total);
		probabilities.add(val4 /total);
		probabilities.add(val5 /total);
		probabilities.add(val6 /total);
		probabilities.add(val7 /total);
		
		double probMass = 0.0;
		for(int i = 0; i< ObstacleQUtilities2.NUM_ACTIONS; i++) {
			probMass += probabilities.get(i);
			if (probMass >= r) {
				return i;
			}
		}
		return -1;
	}
	
	public int randomAction() {
		return (int) Math.random() * ObstacleQUtilities2.NUM_ACTIONS;
	}
	
	public static int XYtoState(double x, double y) {
		int state = -1;
		int row = (int) (y/TILE_SIZE);
		int column = (int) (x/TILE_SIZE);
		state = row*ROW_LENGTH + column;
		return state;
	}
}
