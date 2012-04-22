package qLearning;

import java.util.ArrayList;
import java.util.List;

public class QUtilities3 {
	public static int ROW_LENGTH = 30;
	public static final double TILE_SIZE = 600/ROW_LENGTH;
	public static final int NUM_STATES = ROW_LENGTH * ROW_LENGTH;
	public static final int NUM_ACTIONS = 8;
	public static final int NUM_EPISODES = 100000;
	public static final int GOAL_STATE = 329;
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
	public static final boolean debugFlag = true;
	
	public static void debug(String outVal){
		if (debugFlag) {
			System.out.println(outVal);
		}
	}
	
	public static int[] QtoPolicy(double[][] QTable) {
		int[] policy = new int[NUM_STATES];
		for(int i=0; i< QUtilities3.NUM_STATES; i++) {
			policy[i] = maxA(i, QTable);
				}
		return policy;
	}
	
	public static double[][] generateQTable(int goal) {
		return generateQTable(goal, -1);
	}
	
	public static double[][] generateQTable(int goal, int current) {
		return generateQTable(goal, current, 30, QUtilities.NUM_EPISODES);
	}
	
	public static double[][] generateQTable(int goal, int current, int rows) {
		return generateQTable(goal, current, rows, QUtilities.NUM_EPISODES);
	}
	
	public static double[][] generateQTable(int goal, int current, int rowLength, int episodes) {
		ROW_LENGTH = rowLength;
		if (debugFlag) {
			System.out.println("Initial row length: " + ROW_LENGTH);
		}
		double[][] Q = new double[QUtilities3.NUM_STATES][QUtilities3.NUM_ACTIONS];
		for(int episode = 0; episode < episodes; episode++) {
			int T = 11;
			int s;
			if (current == -1) {
				s = (int)Math.random() * QUtilities3.NUM_STATES;
			} else {
				s = current;
			}
			int sNext;
//			int a = epsilonPickAction(s, Q, 0.0);
			int a = noisyBoltzmannPickAction(s, Q, T);
			if (debugFlag) {
				System.out.println(a);
			}
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
	
	public static void updateQ(double[][] QTable, int curState, int curAction, double r, int nextState){
		double curQ = QTable[curState][curAction];
		QTable[curState][curAction] = curQ + (QUtilities3.alpha * (r + QUtilities3.gamma * (maxAQ(nextState, QTable)- curQ)));
	}
	
	public static int maxA(int state, double[][] Q) {
		int maxA = -1;
		double curMax = -1000000; 
		for(int j=0; j<QUtilities3.NUM_ACTIONS; j++) {
			if (Q[state][j] > curMax) {
				maxA = j;
				curMax = Q[state][j];
			}
		}
		return maxA;
	}
	
	public static double maxAQ(int nextState, double[][] QTable){
		double maxQ = -1000;
		for (int i=0; i < QUtilities3.NUM_ACTIONS; i++) {
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
			return -100;
		} else {
			return - Math.sqrt(Math.pow((double) stateToX(goal) - stateToX(state), 2.0) + Math.pow((stateToY(goal) - stateToY(state)), 2.0));
		}
	}
	
	public static int transitionResult(int state, int action){
		switch(action){
		case QUtilities3.ACTION_NORTH:
			if (! nextToTopWall(state)) {
				return state + QUtilities3.ROW_LENGTH;
			}else {
				return state;
			}
		case QUtilities3.ACTION_NORTHEAST:
			if (! nextToTopWall(state) && ! nextToRightWall(state)) {
				return state + QUtilities3.ROW_LENGTH + 1;
			} else {
				return state;
			}
		case QUtilities3.ACTION_EAST:
			if (! nextToRightWall(state)) {
				return state + 1;
			} else {
				return state;
			}
		case QUtilities3.ACTION_SOUTHEAST:
			if (! nextToBottomWall(state) && ! nextToRightWall(state)) {
				return state - QUtilities3.ROW_LENGTH + 1;
			} else {
				return state;
			}
		case QUtilities3.ACTION_SOUTH:
			if (! nextToBottomWall(state)) {
				return state - QUtilities3.ROW_LENGTH;
			} else {
				return state;
			}
		case QUtilities3.ACTION_SOUTHWEST:
			if (! nextToLeftWall(state) && ! nextToBottomWall(state)) {
				return state - QUtilities3.ROW_LENGTH -1;
			} else {
				return state;
			}
		case QUtilities3.ACTION_WEST:
			if (! nextToLeftWall(state)) {
				return state -1;
			} else {
				return state;
			}
		case QUtilities3.ACTION_NORTHWEST:
			if (! nextToLeftWall(state) && ! nextToTopWall(state)) {
				return state + QUtilities3.ROW_LENGTH -1;
			} else {
				return state;
			}
		}
		return state;
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
		for(int i = 0; i< QUtilities3.NUM_ACTIONS; i++) {
			probMass += probabilities.get(i);
			if (probMass >= r) {
				return i;
			}
		}
		return -1;
	}
	
	public int randomAction() {
		return (int) Math.random() * QUtilities3.NUM_ACTIONS;
	}
	
	public static int XYtoState(double x, double y) {
		int state = -1;
		int row = (int) (y/TILE_SIZE);
		int column = (int) (x/TILE_SIZE);
		state = row*ROW_LENGTH + column;
		return state;
	}
}
