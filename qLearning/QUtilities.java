package qLearning;

import java.util.ArrayList;
import java.util.List;

public class QUtilities {
	public static final int NUM_STATES = 900;
	public static final int ROW_LENGTH = 30;
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
	
	public static int[] QtoPolicy(double[][] QTable) {
		int[] policy = new int[NUM_STATES];
		for(int i=0; i< QUtilities.NUM_STATES; i++) {
			policy[i] = maxA(i, QTable);
				}
		return policy;
	}
	
	public static double[][] generateQTable(int goal) {
		double[][] Q = new double[QUtilities.NUM_STATES][QUtilities.NUM_ACTIONS];
		for(int episode = 0; episode < NUM_EPISODES; episode++) {
			int T = 11;
			int s = (int)Math.random() * 600;
			int sNext;
			int a = epsilonPickAction(s, Q, 0.0);
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
				a = epsilonPickAction(s, Q, QUtilities.epsilon);
			}
		}
		
		return Q;
		
	}
	
	public static void updateQ(double[][] QTable, int curState, int curAction, double r, int nextState){
		double curQ = QTable[curState][curAction];
		QTable[curState][curAction] = curQ + (QUtilities.alpha * (r + QUtilities.gamma * (maxAQ(nextState, QTable)- curQ)));
	}
	
	public static int maxA(int state, double[][] Q) {
		int maxA = -1;
		double curMax = -1000000; 
		for(int j=0; j<QUtilities.NUM_ACTIONS; j++) {
			if (Q[state][j] > curMax) {
				maxA = j;
				curMax = Q[state][j];
			}
		}
		return maxA;
	}
	
	public static double maxAQ(int nextState, double[][] QTable){
		double maxQ = -1000;
		for (int i=0; i < QUtilities.NUM_ACTIONS; i++) {
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
			return - Math.sqrt(Math.pow((double) stateToX(goal) - stateToX(state), 2.0) + Math.pow((double) (stateToY(goal) - stateToY(state)), 2.0));
		}
	}
	
	public static int transitionResult(int state, int action){
		switch(action){
		case QUtilities.ACTION_NORTH:
			if (! nextToTopWall(state)) {
				return state + QUtilities.ROW_LENGTH;
			}else {
				return state;
			}
		case QUtilities.ACTION_NORTHEAST:
			if (! nextToTopWall(state) && ! nextToRightWall(state)) {
				return state + QUtilities.ROW_LENGTH + 1;
			} else {
				return state;
			}
		case QUtilities.ACTION_EAST:
			if (! nextToRightWall(state)) {
				return state + 1;
			} else {
				return state;
			}
		case QUtilities.ACTION_SOUTHEAST:
			if (! nextToBottomWall(state) && ! nextToRightWall(state)) {
				return state - QUtilities.ROW_LENGTH + 1;
			} else {
				return state;
			}
		case QUtilities.ACTION_SOUTH:
			if (! nextToBottomWall(state)) {
				return state - QUtilities.ROW_LENGTH;
			} else {
				return state;
			}
		case QUtilities.ACTION_SOUTHWEST:
			if (! nextToLeftWall(state) && ! nextToBottomWall(state)) {
				return state - QUtilities.ROW_LENGTH -1;
			} else {
				return state;
			}
		case QUtilities.ACTION_WEST:
			if (! nextToLeftWall(state)) {
				return state -1;
			} else {
				return state;
			}
		case QUtilities.ACTION_NORTHWEST:
			if (! nextToLeftWall(state) && ! nextToTopWall(state)) {
				return state + QUtilities.ROW_LENGTH -1;
			} else {
				return state;
			}
		}
		return state;
	}
	
	public static boolean nextToTopWall(int state) {
		if ((state/30) >= 29){
			return true;
		} else {
			return false;
		}
	}
	
	public static boolean nextToBottomWall(int state) {
		if ((state/30) <= 0){
			return true;
		} else {
			return false;
		}
	}
	
	public static boolean nextToRightWall(int state) {
		if ((state % 30) >= 29){
			return true;
		} else {
			return false;
		}
	}
	
	public static boolean nextToLeftWall(int state) {
		if ((state % 30) <= 0){
			return true;
		} else {
			return false;
		}
	}
	
	public static int stateToX(int state) {
		return state % 30;
	}
	
	public static int stateToY(int state) {
		return state / 30;
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
		for(int i = 0; i< QUtilities.NUM_ACTIONS; i++) {
			probMass += probabilities.get(i);
			if (probMass >= r) {
				return i;
			}
		}
		return -1;
	}
	
	public int randomAction() {
		return (int) Math.random() * QUtilities.NUM_ACTIONS;
	}
}
