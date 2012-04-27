package mdp;
import java.util.*;
/*
 * Abstract: Implements methods to create/manage/implement a Markov Decision Process over a disctrete state-space.
 * Author: TJ Collins
 * Notes: The utility functions provided below handle the creation of optimal policies through value iteration, the 
 * tiling of state space given continuous coordinate values, the definition of transition probabilities, and the 
 * definition of rewards, among other things. Below, you will also find my real-time extensions to value iteration
 */
public class MDPUtility {
	public static final int NUM_STATES = 400;
	public static final int NUM_ACTIONS = 8;
	public static final double DISCOUNT_FACTOR = 0.9;
	public static final double RESIDUAL = 0.01;
	public static final int ACTION_NORTH = 0;
	public static final int ACTION_SOUTH = 1;
	public static final int ACTION_EAST = 2;
	public static final int ACTION_WEST = 3;
	public static final int ACTION_NORTHEAST = 4;
	public static final int ACTION_NORTHWEST = 5;
	public static final int ACTION_SOUTHEAST = 6;
	public static final int ACTION_SOUTHWEST = 7;
	public static final int NUM_STATES_IN_ROW = 20;
	public static final int TILE_SIZE = 30;
	public static final double GRID_SIZE_X = 600.0;
	public static final double GRID_SIZE_Y = 600.0;
	public static final int GOAL_STATE = 0;
	
	/*
	 * Implementation of value iteration over a discrete state space. Returns q_table of Q-values associated with every possible
	 * state-action pair
	 */
	public static double[][] valueIteration (double[][][] transitions, double[][] rewards) {
		double[] value = new double[NUM_STATES];
		double[] valuePrevious = new double[NUM_STATES];
		double[][] q_table = new double[NUM_STATES][NUM_ACTIONS];
		int count = 0;
		//While the vector norm of the difference between the value and previous value vector is larger larger than the Bellman residual
		while (vectorNormOfDifference(value, valuePrevious) > RESIDUAL || count == 0) {
			//Set valuePrevious vector to be equal to value vector
			for (int i = 0; i < NUM_STATES; i++) {
				valuePrevious[i] = value[i];
			}
			//Loop through all state/action pairs
			for (int s = 0; s < NUM_STATES; s++) {
				for (int a = 0; a < NUM_ACTIONS; a++) {
					double backup = 0.0;
					//Do a Bellman backup here over any states we can transition to from the state/action pair we are looking at
					for (int sprime = 0; sprime < NUM_STATES; sprime++) {
						backup += transitions[s][sprime][a]*valuePrevious[sprime];
					}
					//Set the q-value for the state action pair
					q_table[s][a] = rewards[s][a] + DISCOUNT_FACTOR*backup;
				}
				double maxq = -Double.MAX_VALUE;
				for (int k = 0; k<NUM_ACTIONS; k++) {
					if (q_table[s][k] > maxq) {
						maxq = q_table[s][k];
					}
				}
				//value of state is max q-value 
				value[s] = maxq;
			}
			count++;
		}
		System.out.print("Convergence in " + count + " iterations");
		System.out.println();
		return q_table;
	}
	
	/*
	 * Real time value iteration function that modifies the given q_table based on the recent change to the goal state
	 */
	public static double[][] valueIterationRealTime(int goal_state, int previous_goal_state, double[][][] transitions, double[][] rewards, double[][] q_table) {
		int num_states_to_update = 2*NUM_ACTIONS + 2 + (NUM_ACTIONS*NUM_ACTIONS)*2;
		double[][] new_q = q_table;
		double[] value = new double[NUM_STATES];
		double[] valuePrevious = new double[NUM_STATES];
		int[] states_to_update = new int[num_states_to_update];
		int count = 0;
		int[] states_surrounding_new_goal = getSurroundingStates(goal_state);
		int[] states_surrounding_old_goal = getSurroundingStates(previous_goal_state);
		//Update the states surrounding the new goal and the old goal
		for (int s = 0; s < NUM_ACTIONS; s++) {
			states_to_update[s] = states_surrounding_new_goal[s];
		}
		
		for (int s = NUM_ACTIONS; s < 2*NUM_ACTIONS; s++) {
			states_to_update[s] = states_surrounding_old_goal[s - NUM_ACTIONS];
		}
		
		//Update the state surrounding the states surrounding the old goal and the states surrounding the states surrounding the new goal
		int index = 2*NUM_ACTIONS + 2;
		for (int s = 0; s < NUM_ACTIONS; s++) {
			if (states_surrounding_new_goal[s] != -1) {
				//states_to_update[index] = states_surrounding_new_goal[s];
				//index++;
				int[] surround = getSurroundingStates(states_surrounding_new_goal[s]);
				for (int s2 = 0; s2< NUM_ACTIONS; s2++) {
					if (surround[s2] != -1) {
						states_to_update[index] = surround[s2];
					} else {
						states_to_update[index] = -1;
					}
					index++;
				}
			} else {
				//states_to_update[index] = -1;
				//index++;
				for (int s2 = 0; s2< NUM_ACTIONS; s2++) {
					states_to_update[index] = -1;
					index++;
				}
			}
			
			
			if (states_surrounding_old_goal[s] != -1) {
				//states_to_update[index] = states_surrounding_old_goal[s];
				//index++;
				int[] surround = getSurroundingStates(states_surrounding_old_goal[s]);
				for (int s2 = 0; s2< NUM_ACTIONS; s2++) {
					if (surround[s2] != -1) {
						states_to_update[index] = surround[s2];
					} else {
						states_to_update[index] = -1;
					}
					index++;
				}
			} else {
				//states_to_update[index] = -1;
				//index++;
				for (int s2 = 0; s2< NUM_ACTIONS; s2++) {
					states_to_update[index] = -1;
					index++;
				}
			}
			
		}
		
		states_to_update[NUM_ACTIONS*2] = goal_state;
		states_to_update[NUM_ACTIONS*2 + 1] = previous_goal_state;
		
		while (vectorNormOfDifference(value, valuePrevious) > RESIDUAL || count == 0) {
			//Update the value vector only in the appropriate places
			for (int i = 0; i < num_states_to_update; i++) {
				if(states_to_update[i] != -1) valuePrevious[states_to_update[i]] = value[states_to_update[i]];
			}
			//Only iterate over the number of states we need to update
			for (int s = 0; s < num_states_to_update; s++) {
				for (int a = 0; a < NUM_ACTIONS; a++) {
					double backup = 0.0;
					//Perform the bellman backup. we know that we can only transition to this state from the surrounding ones
					//so this is also how we save time.
					int[] sur = getSurroundingStates(states_to_update[s]);
					for (int sprime = 0; sprime < NUM_ACTIONS; sprime++) {
						if (states_to_update[s] != -1 && sur[sprime] != -1) backup += transitions[states_to_update[s]][sur[sprime]][a]*valuePrevious[sur[sprime]];
					}
					//Update q-values in the appropriate places
					if (states_to_update[s] != -1) q_table[states_to_update[s]][a] = rewards[states_to_update[s]][a] + DISCOUNT_FACTOR*backup;
				}
				//Update policy in appropriate places
				double maxq = -Double.MAX_VALUE;
				if (states_to_update[s] != -1) {
					for (int k = 0; k<NUM_ACTIONS; k++) {
						if (q_table[states_to_update[s]][k] > maxq) {
							maxq = q_table[states_to_update[s]][k];
						}
					}
				}
				if (states_to_update[s] != -1 && maxq > Double.MIN_VALUE) value[states_to_update[s]] = maxq;
			}
			count++;
		}
		//System.out.print("Real time Convergence in " + count + " iterations");
		//System.out.println();
		return new_q;
	}
	
	/*
	 * A failed experiment in doing the same real-time update but taking obstacles into account. The method converges and converges in a fewer number of
	 * iterations than the full value iteration procedure, but the results were non-intuitive.
	 */
	public static double[][] valueIterationRealTimeWithObstacles(int goal_state, int previous_goal_state, double[][][] transitions, double[][] rewards, double[][] q_table, Map<String, Integer> obstacles, Map<String, Integer> obstacles_old) {
		int num_states_to_update = 2*NUM_ACTIONS + 2 + (NUM_ACTIONS + NUM_ACTIONS*NUM_ACTIONS)*obstacles.size() + (NUM_ACTIONS + NUM_ACTIONS*NUM_ACTIONS)*obstacles_old.size() + NUM_ACTIONS + NUM_ACTIONS;
		double[][] new_q = q_table;
		double[] value = new double[NUM_STATES];
		double[] valuePrevious = new double[NUM_STATES];
		int[] states_to_update;
		int obstacles_size = 0;
			states_to_update = new int[num_states_to_update];
		int count = 0;
		int[] states_surrounding_new_goal = getSurroundingStates(goal_state);
		int[] states_surrounding_old_goal = getSurroundingStates(previous_goal_state);
		int obstacles_index = 2*NUM_ACTIONS + 2;
			for (Map.Entry<String, Integer> entry : obstacles.entrySet()) {
				int[] states_surrounding_obstacle = getSurroundingStates(entry.getValue());
				for (int s = 0; s < NUM_ACTIONS; s++) {
					if (states_surrounding_obstacle[s] != -1) {
						states_to_update[obstacles_index] = states_surrounding_obstacle[s];
						obstacles_index++;
						int[] states_surrounding_surrounding_obstacles = getSurroundingStates(states_surrounding_obstacle[s]);
						for (int s2 = 0; s2 < NUM_ACTIONS; s2++) {
							if (states_surrounding_surrounding_obstacles[s2] != -1) {
								states_to_update[obstacles_index] = states_surrounding_surrounding_obstacles[s2];
							} else {
								states_to_update[obstacles_index] = -1;
							}
							obstacles_index++;
						}
					} else {
						for (int s2 = 0; s2 < NUM_ACTIONS; s2++) {
							states_to_update[obstacles_index] = -1;
							obstacles_index++;
						}
					}
				}
				states_to_update[obstacles_index] = entry.getValue();
				obstacles_index++;

			}
			
			for (Map.Entry<String, Integer> entry : obstacles_old.entrySet()) {
				int[] states_surrounding_obstacle = getSurroundingStates(entry.getValue());
				for (int s = 0; s < NUM_ACTIONS; s++) {
					if (states_surrounding_obstacle[s] != -1) {
						states_to_update[obstacles_index] = states_surrounding_obstacle[s];
						obstacles_index++;
						int[] states_surrounding_surrounding_obstacles = getSurroundingStates(states_surrounding_obstacle[s]);
						for (int s2 = 0; s2 < NUM_ACTIONS; s2++) {
							if (states_surrounding_surrounding_obstacles[s2] != -1) {
								states_to_update[obstacles_index] = states_surrounding_surrounding_obstacles[s2];
							} else {
								states_to_update[obstacles_index] = -1;
							}
							obstacles_index++;
						}
					} else {
						for (int s2 = 0; s2 < NUM_ACTIONS; s2++) {
							states_to_update[obstacles_index] = -1;
							obstacles_index++;
						}
					}
				}
				states_to_update[obstacles_index] = entry.getValue();
				obstacles_index++;

			}
			
		

		for (int s = 0; s < NUM_ACTIONS; s++) {
			states_to_update[s] = states_surrounding_new_goal[s];
		}

		for (int s = NUM_ACTIONS; s < 2*NUM_ACTIONS; s++) {
			states_to_update[s] = states_surrounding_old_goal[s - NUM_ACTIONS];
		}

		states_to_update[NUM_ACTIONS*2] = goal_state;
		states_to_update[NUM_ACTIONS*2 + 1] = previous_goal_state;

		while (vectorNormOfDifference(value, valuePrevious) > RESIDUAL || count == 0) {
			for (int i = 0; i < 2*NUM_ACTIONS + 2 + (NUM_ACTIONS + NUM_ACTIONS*NUM_ACTIONS*obstacles_size); i++) {
				if(states_to_update[i] != -1) valuePrevious[states_to_update[i]] = value[states_to_update[i]];
			}
			for (int s = 0; s < 2*NUM_ACTIONS + 2 + (NUM_ACTIONS + NUM_ACTIONS*NUM_ACTIONS*obstacles_size); s++) {
				for (int a = 0; a < NUM_ACTIONS; a++) {
					double backup = 0.0;
					int[] sur = getSurroundingStates(states_to_update[s]);
					for (int sprime = 0; sprime < NUM_ACTIONS; sprime++) {
						if (states_to_update[s] != -1 && sur[sprime] != -1) backup += transitions[states_to_update[s]][sur[sprime]][a]*valuePrevious[sur[sprime]];
					}
					if (states_to_update[s] != -1) q_table[states_to_update[s]][a] = rewards[states_to_update[s]][a] + DISCOUNT_FACTOR*backup;
				}
				double maxq = -Double.MAX_VALUE;
				if (states_to_update[s] != -1) {
					for (int k = 0; k<NUM_ACTIONS; k++) {
						if (q_table[states_to_update[s]][k] > maxq) {
							maxq = q_table[states_to_update[s]][k];
						}
					}
				}
				if (states_to_update[s] != -1 && maxq > Double.MIN_VALUE) value[states_to_update[s]] = maxq;
			}
			count++;
		}
		System.out.print("Real time Convergence in " + count + " iterations");
		System.out.println();
		return new_q;
	}
	
	/*
	 * This method updates our reward function only in the proper locations
	 */
	public static double[][] updateRewardsRealTime(int goal_state, int previous_goal_state, double[][][] transitions, double[][] rewards) {
		double[][] new_rewards = rewards;
		int[] states_surrounding_new_goal = getSurroundingStates(goal_state);
		int[] states_surrounding_old_goal = getSurroundingStates(previous_goal_state);
		
		//Update rewards in the states surrounding the new goal and the states surrounding the states surrounding the new goal
		for (int s = 0; s < NUM_ACTIONS; s++) {
			if (states_surrounding_new_goal[s] != -1) {
				for (int a = 0; a< NUM_ACTIONS; a++) {
					new_rewards[states_surrounding_new_goal[s]][a] = getRewardForGoal(states_surrounding_new_goal[s], a, transitions, goal_state);
				}
				int[] surrounding_states = getSurroundingStates(states_surrounding_new_goal[s]);
				for (int s2 = 0; s2 < NUM_ACTIONS; s2++) {
					for (int a = 0; a<NUM_ACTIONS; a++) {
						if (surrounding_states[s2] != -1) {
							new_rewards[surrounding_states[s2]][a] = getRewardForGoal(surrounding_states[s2], a, transitions, goal_state);
						}
					}
				}
			}
		}
		//update the rewards for the new goal state
		for (int a = 0; a < NUM_ACTIONS; a++) {
			new_rewards[goal_state][a] = getRewardForGoal(goal_state, a, transitions, goal_state);
		}
		
		//Update rewards in the states surrounding the old goal and the states surrounding the states surrounding the old goal
		for (int s = 0; s < NUM_ACTIONS; s++) {
			if (states_surrounding_old_goal[s] != -1) {
				for (int a = 0; a< NUM_ACTIONS; a++) {
					new_rewards[states_surrounding_old_goal[s]][a] = getRewardForGoal(states_surrounding_old_goal[s], a, transitions, goal_state);
				}
				int[] surrounding_states = getSurroundingStates(states_surrounding_old_goal[s]);
				for (int s2 = 0; s2 < NUM_ACTIONS; s2++) {
					for (int a = 0; a<NUM_ACTIONS; a++) {
						if (surrounding_states[s2] != -1) {
							new_rewards[surrounding_states[s2]][a] = getRewardForGoal(surrounding_states[s2], a, transitions, goal_state);
						}
					}
				}
			}
		}
		//update rewards for the old goal state
		for (int a = 0; a < NUM_ACTIONS; a++) {
			new_rewards[previous_goal_state][a] = getRewardForGoal(previous_goal_state, a, transitions, goal_state);
		}
		return new_rewards;
	}
	
	/*
	 * Updating rewards in real time whilst taking obstacles into account. This method seems to be working.
	 */
	public static double[][] updateRewardsRealTimeWithObstacles(int goal_state, int previous_goal_state, double[][][] transitions, double[][] rewards, Map<String,Integer> obstacles, Map<String,Integer> obstacles_old) {
		double[][] new_rewards = rewards;
		int[] states_surrounding_new_goal = getSurroundingStates(goal_state);
		int[] states_surrounding_old_goal = getSurroundingStates(previous_goal_state);
			for (Map.Entry<String, Integer> entry : obstacles.entrySet()) {
				int[] states_surrounding_obstacle = getSurroundingStates(entry.getValue());
				for (int s = 0; s < NUM_ACTIONS; s++) {
					if (states_surrounding_obstacle[s] != -1) {
						for (int a = 0; a< NUM_ACTIONS; a++) {
							new_rewards[states_surrounding_obstacle[s]][a] = getRewardForGoalWithObstacles(states_surrounding_obstacle[s], a, transitions, goal_state, obstacles);
						}
						int[] states_surrounding_surrounding_obstacles = getSurroundingStates(states_surrounding_obstacle[s]);
						for (int s2 = 0; s2 < NUM_ACTIONS; s2++) {
							if (states_surrounding_surrounding_obstacles[s2] != -1) {
								for (int a = 0; a< NUM_ACTIONS; a++) {
									new_rewards[states_surrounding_surrounding_obstacles[s2]][a] = getRewardForGoalWithObstacles(states_surrounding_surrounding_obstacles[s2], a, transitions, goal_state, obstacles);
								}
							}
						}
					}
				}
				for (int a = 0; a<NUM_ACTIONS; a++) {
					new_rewards[entry.getValue()][a] = getRewardForGoalWithObstacles(entry.getValue(), a, transitions, goal_state, obstacles);
				}

			}
			
			for (Map.Entry<String, Integer> entry : obstacles.entrySet()) {
				int[] states_surrounding_obstacle = getSurroundingStates(entry.getValue());
				for (int s = 0; s < NUM_ACTIONS; s++) {
					if (states_surrounding_obstacle[s] != -1) {
						for (int a = 0; a< NUM_ACTIONS; a++) {
							new_rewards[states_surrounding_obstacle[s]][a] = getRewardForGoalWithObstacles(states_surrounding_obstacle[s], a, transitions, goal_state, obstacles);
						}
						int[] states_surrounding_surrounding_obstacles = getSurroundingStates(states_surrounding_obstacle[s]);
						for (int s2 = 0; s2 < NUM_ACTIONS; s2++) {
							if (states_surrounding_surrounding_obstacles[s2] != -1) {
								for (int a = 0; a< NUM_ACTIONS; a++) {
									new_rewards[states_surrounding_surrounding_obstacles[s2]][a] = getRewardForGoalWithObstacles(states_surrounding_surrounding_obstacles[s2], a, transitions, goal_state, obstacles);
								}
							}
						}
					}
				}
				for (int a = 0; a<NUM_ACTIONS; a++) {
					new_rewards[entry.getValue()][a] = getRewardForGoalWithObstacles(entry.getValue(), a, transitions, goal_state, obstacles);
				} 
			}
		

		
		for (int s = 0; s < NUM_ACTIONS; s++) {
			if (states_surrounding_new_goal[s] != -1) {
				for (int a = 0; a< NUM_ACTIONS; a++) {
					new_rewards[states_surrounding_new_goal[s]][a] = getRewardForGoalWithObstacles(states_surrounding_new_goal[s], a, transitions, goal_state, obstacles);
				}
			}
		}
		for (int a = 0; a < NUM_ACTIONS; a++) {
			new_rewards[goal_state][a] = getRewardForGoalWithObstacles(goal_state, a, transitions, goal_state, obstacles);
		}

		for (int s = 0; s < NUM_ACTIONS; s++) {
			if (states_surrounding_old_goal[s] != -1) {
				for (int a = 0; a< NUM_ACTIONS; a++) {
					new_rewards[states_surrounding_old_goal[s]][a] = getRewardForGoalWithObstacles(states_surrounding_old_goal[s], a, transitions, goal_state, obstacles);
				}
			}
		}
		for (int a = 0; a < NUM_ACTIONS; a++) {
			new_rewards[previous_goal_state][a] = getRewardForGoalWithObstacles(previous_goal_state, a, transitions, goal_state, obstacles);
		}
		return new_rewards;
	}

	/*
	 * Update the policy only in the appropriate locations after  we have made the necessary adjustments to the q-table and the rewards in
	 * real time
	 */
	public static int[] updatePolicyRealTime(int[] policy, double[][] q_table, int goal_state, int previous_goal_state) {
		int num_states_to_update = 2*NUM_ACTIONS + 2 + (NUM_ACTIONS*NUM_ACTIONS)*2;
		int[] new_policy = policy;
		int[] states_to_update = new int[num_states_to_update];
		int[] states_surrounding_new_goal = getSurroundingStates(goal_state);
		int[] states_surrounding_old_goal = getSurroundingStates(previous_goal_state);
		
		//Update states surrounding new goal
		for (int s = 0; s < NUM_ACTIONS; s++) {
			states_to_update[s] = states_surrounding_new_goal[s];
		}
		
		//Update states surrounding old goal
		for (int s = NUM_ACTIONS; s < 2*NUM_ACTIONS; s++) {
			states_to_update[s] = states_surrounding_old_goal[s - NUM_ACTIONS];
		}
		
		//update old and new goal states
		states_to_update[NUM_ACTIONS*2] = goal_state;
		states_to_update[NUM_ACTIONS*2 + 1] = previous_goal_state;
		
		//Update states surrounding the states surrounding the old and new goal states
		int index = 2*NUM_ACTIONS + 2;
		for (int s = 0; s < NUM_ACTIONS; s++) {
			if (states_surrounding_new_goal[s] != -1) {
				//states_to_update[index] = states_surrounding_new_goal[s];
				//index++;
				int[] surround = getSurroundingStates(states_surrounding_new_goal[s]);
				for (int s2 = 0; s2< NUM_ACTIONS; s2++) {
					if (surround[s2] != -1) {
						states_to_update[index] = surround[s2];
					} else {
						states_to_update[index] = -1;
					}
					index++;
				}
			} else {
				//states_to_update[index] = -1;
				//index++;
				for (int s2 = 0; s2< NUM_ACTIONS; s2++) {
					states_to_update[index] = -1;
					index++;
				}
			}
			
			
			if (states_surrounding_old_goal[s] != -1) {
				//states_to_update[index] = states_surrounding_old_goal[s];
				//index++;
				int[] surround = getSurroundingStates(states_surrounding_old_goal[s]);
				for (int s2 = 0; s2< NUM_ACTIONS; s2++) {
					if (surround[s2] != -1) {
						states_to_update[index] = surround[s2];
					} else {
						states_to_update[index] = -1;
					}
					index++;
				}
			} else {
				//states_to_update[index] = -1;
				//index++;
				for (int s2 = 0; s2< NUM_ACTIONS; s2++) {
					states_to_update[index] = -1;
					index++;
				}
			}
			
		}
		
		for (int s = 0; s < num_states_to_update; s++) {
			double maxq = -Double.MAX_VALUE;
			int maxa = -1;
			if (states_to_update[s] != -1) {
			for (int a = 0; a<NUM_ACTIONS; a++) {
				if (q_table[states_to_update[s]][a] > maxq) {
					maxq = q_table[states_to_update[s]][a];
					maxa = a;
				}
			}
			new_policy[states_to_update[s]] = maxa;
			}
		}
		return new_policy;
	}
	
	/*
	 * An attempt to update the policy in real time while taking obstacles into account. 
	 */
	public static int[] updatePolicyRealTimeWithObstacles(int[] policy, double[][] q_table, int goal_state, int previous_goal_state, Map<String, Integer> obstacles, Map<String, Integer> obstacles_old) {
		int num_states_to_update = 2*NUM_ACTIONS + 2 + (NUM_ACTIONS + NUM_ACTIONS*NUM_ACTIONS)*obstacles.size() + (NUM_ACTIONS + NUM_ACTIONS*NUM_ACTIONS)*obstacles_old.size() + NUM_ACTIONS + NUM_ACTIONS;
		int[] new_policy = policy;
		int[] states_to_update = new int[num_states_to_update];
		int[] states_surrounding_new_goal = getSurroundingStates(goal_state);
		int[] states_surrounding_old_goal = getSurroundingStates(previous_goal_state);
		
		int obstacles_index = 2*NUM_ACTIONS + 2;
			for (Map.Entry<String, Integer> entry : obstacles.entrySet()) {
				int[] states_surrounding_obstacle = getSurroundingStates(entry.getValue());
				for (int s = 0; s < NUM_ACTIONS; s++) {
					if (states_surrounding_obstacle[s] != -1) {
						states_to_update[obstacles_index] = states_surrounding_obstacle[s];
						obstacles_index++;
						int[] states_surrounding_surrounding_obstacles = getSurroundingStates(states_surrounding_obstacle[s]);
						for (int s2 = 0; s2 < NUM_ACTIONS; s2++) {
							if (states_surrounding_surrounding_obstacles[s2] != -1) {
								states_to_update[obstacles_index] = states_surrounding_surrounding_obstacles[s2];
							} else {
								states_to_update[obstacles_index] = -1;
							}
							obstacles_index++;
						}
					} else {
						for (int s2 = 0; s2 < NUM_ACTIONS; s2++) {
							states_to_update[obstacles_index] = -1;
							obstacles_index++;
						}
					}
				}
				states_to_update[obstacles_index] = entry.getValue();
				obstacles_index++;
			}
		
			for (Map.Entry<String, Integer> entry : obstacles_old.entrySet()) {
				int[] states_surrounding_obstacle = getSurroundingStates(entry.getValue());
				for (int s = 0; s < NUM_ACTIONS; s++) {
					if (states_surrounding_obstacle[s] != -1) {
						states_to_update[obstacles_index] = states_surrounding_obstacle[s];
						obstacles_index++;
						int[] states_surrounding_surrounding_obstacles = getSurroundingStates(states_surrounding_obstacle[s]);
						for (int s2 = 0; s2 < NUM_ACTIONS; s2++) {
							if (states_surrounding_surrounding_obstacles[s2] != -1) {
								states_to_update[obstacles_index] = states_surrounding_surrounding_obstacles[s2];
							} else {
								states_to_update[obstacles_index] = -1;
							}
							obstacles_index++;
						}
					} else {
						for (int s2 = 0; s2 < NUM_ACTIONS; s2++) {
							states_to_update[obstacles_index] = -1;
							obstacles_index++;
						}
					}
				}
				states_to_update[obstacles_index] = entry.getValue();
				obstacles_index++;
			}
		
		for (int s = 0; s < NUM_ACTIONS; s++) {
			states_to_update[s] = states_surrounding_new_goal[s];
		}
		
		for (int s = NUM_ACTIONS; s < 2*NUM_ACTIONS; s++) {
			states_to_update[s] = states_surrounding_old_goal[s - NUM_ACTIONS];
		}
		
		states_to_update[NUM_ACTIONS*2] = goal_state;
		states_to_update[NUM_ACTIONS*2 + 1] = previous_goal_state;
		
		for (int s = 0; s < num_states_to_update; s++) {
			double maxq = -Double.MAX_VALUE;
			int maxa = -1;
			if (states_to_update[s] != -1) {
			for (int a = 0; a<NUM_ACTIONS; a++) {
				if (q_table[states_to_update[s]][a] > maxq) {
					maxq = q_table[states_to_update[s]][a];
					maxa = a;
				}
			}
			new_policy[states_to_update[s]] = maxa;
			}
		}
		return new_policy;
	}
	
	public static int[] getSurroundingStates(int state) {
		int[] surrounding = new int[8];
		
		if (state % NUM_STATES_IN_ROW != 0 && state - 1 >= 0)  {
			surrounding[0] = state - 1;
		} else {
			surrounding[0] = -1;
		}
		
		if (state % NUM_STATES_IN_ROW != NUM_STATES_IN_ROW - 1 && state + 1 < NUM_STATES) {
			surrounding[1] = state + 1;
		} else {
			surrounding[1] = -1;
		}
		
		if (state + NUM_STATES_IN_ROW < NUM_STATES) {
			surrounding[2] = state + NUM_STATES_IN_ROW;
		} else {
			surrounding[2] = -1;
		}
		
		if (state - NUM_STATES_IN_ROW >= 0) {
			surrounding[3] = state - NUM_STATES_IN_ROW;
		} else {
			surrounding[3] = -1;
		}
		
		if (state % NUM_STATES_IN_ROW != 0 && state + NUM_STATES_IN_ROW - 1 < NUM_STATES) {
			surrounding[4] = state + NUM_STATES_IN_ROW - 1;
		} else {
			surrounding[4] = -1;
		}
		
		if (state % NUM_STATES_IN_ROW != 0 && state - NUM_STATES_IN_ROW - 1 >= 0) {
			surrounding[5] = state - NUM_STATES_IN_ROW - 1;
		} else {
			surrounding[5] = -1;
		}
		
		if (state % NUM_STATES_IN_ROW != NUM_STATES_IN_ROW - 1 && state + NUM_STATES_IN_ROW + 1 < NUM_STATES) {
			surrounding[6] = state + NUM_STATES_IN_ROW + 1;
		} else {
			surrounding[6] = -1;
		}
		
		if (state % NUM_STATES_IN_ROW != NUM_STATES_IN_ROW - 1 && state - NUM_STATES_IN_ROW + 1 >= 0) {
			surrounding[7] = state - NUM_STATES_IN_ROW + 1;
		} else {
			surrounding[7] = -1;
		}
		
		
		return surrounding;
	}
	
	
	/*takes in two vectors (1D arrays) and returns the norm (in the linear algebra sense) of the vector defined by
	 * vectorA- vectorB
	 */
	private static double vectorNormOfDifference(double[] vectorA, double[] vectorB) {
		double norm = 0.0;
		for (int i=0; i<vectorA.length; i++) {
			double newVal = vectorA[i] - vectorB[i];
			norm+= Math.pow(newVal, 2);
		}
		norm = Math.sqrt(norm);
		return norm;
	}
	
	/*takes in two states and an action, and determines the transition probability of moving from state1 to state2 via
	 * the given action
	 */
	public static double getTransitionProbability(int state1, int state2, int action) {
		double reachable = 0.0;
		if (action == ACTION_NORTH && state2 == (state1 + NUM_STATES_IN_ROW)) {
			reachable = 1.0;
		} else if (action == ACTION_SOUTH && state2 == (state1 - NUM_STATES_IN_ROW)) {
			reachable = 1.0;
		} else if (action == ACTION_EAST && state2 == (state1 + 1) && state1 % NUM_STATES_IN_ROW != (NUM_STATES_IN_ROW - 1)) {
			reachable = 1.0;
		} else if (action == ACTION_WEST && state2 == (state1 - 1) && state1 % NUM_STATES_IN_ROW != 0) {
			reachable = 1.0;
		} else if (action == ACTION_NORTHWEST && state2 == (state1 + NUM_STATES_IN_ROW - 1) && state1 % NUM_STATES_IN_ROW != 0) {
			reachable = 1.0;
		} else if (action == ACTION_NORTHEAST && state2 == (state1 + NUM_STATES_IN_ROW + 1) && state1 % NUM_STATES_IN_ROW != NUM_STATES_IN_ROW - 1) {
			reachable = 1.0;
		} else if (action == ACTION_SOUTHWEST && state2 == (state1 - NUM_STATES_IN_ROW - 1) && state1 % NUM_STATES_IN_ROW != 0) {
			reachable = 1.0;
		} else if (action == ACTION_SOUTHEAST && state2 == (state1 - NUM_STATES_IN_ROW + 1) && state1 % NUM_STATES_IN_ROW != NUM_STATES_IN_ROW - 1) {
			reachable = 1.0;
		} else if (state2 == state1) {
			if (state1 % NUM_STATES_IN_ROW == 0) {
				if (action == ACTION_WEST || action == ACTION_NORTHWEST || action == ACTION_SOUTHWEST) {
					reachable = 1.0;
				}
			} 
			if (state1 % NUM_STATES_IN_ROW == NUM_STATES_IN_ROW -1) {
				if (action == ACTION_EAST || action == ACTION_NORTHEAST || action == ACTION_SOUTHEAST) {
					reachable = 1.0;
				}
			} 
			if (state1 < NUM_STATES_IN_ROW) {
				if (action == ACTION_SOUTH || action == ACTION_SOUTHWEST || action == ACTION_SOUTHEAST) {
					reachable = 1.0;
				}
			}
			if ((state1 +  NUM_STATES_IN_ROW) >= NUM_STATES) {
				if (action == ACTION_NORTH || action == ACTION_NORTHWEST || action == ACTION_NORTHEAST) {
					reachable = 1.0;
				}
			}
		}
		return reachable;
	}
	
	/*
	 * Returns transition probability between state1 and state2 by taking action in state1. This transition
	 * probability function is noisy. This is how we tell our robot about the 20% slippage in its actions.
	 */
	public static double getTransitionProbabilityNoisy(int state1, int state2, int action) {
		double reachable = 0.0;
		if (action == ACTION_NORTH && state2 == (state1 + NUM_STATES_IN_ROW)) {
			reachable = 0.8;
		} else if (action == ACTION_SOUTH && state2 == (state1 - NUM_STATES_IN_ROW)) {
			reachable = 0.8;
		} else if (action == ACTION_EAST && state2 == (state1 + 1) && state1 % NUM_STATES_IN_ROW != (NUM_STATES_IN_ROW - 1)) {
			reachable = 0.8;
		} else if (action == ACTION_WEST && state2 == (state1 - 1) && state1 % NUM_STATES_IN_ROW != 0) {
			reachable = 0.8;
		} else if (action == ACTION_NORTHWEST && state2 == (state1 + NUM_STATES_IN_ROW - 1) && state1 % NUM_STATES_IN_ROW != 0) {
			reachable = 0.8;
		} else if (action == ACTION_NORTHEAST && state2 == (state1 + NUM_STATES_IN_ROW + 1) && state1 % NUM_STATES_IN_ROW != NUM_STATES_IN_ROW - 1) {
			reachable = 0.8;
		} else if (action == ACTION_SOUTHWEST && state2 == (state1 - NUM_STATES_IN_ROW - 1) && state1 % NUM_STATES_IN_ROW != 0) {
			reachable = 0.8;
		} else if (action == ACTION_SOUTHEAST && state2 == (state1 - NUM_STATES_IN_ROW + 1) && state1 % NUM_STATES_IN_ROW != NUM_STATES_IN_ROW - 1) {
			reachable = 0.8;
		} else if (action == ACTION_NORTH && state2 == (state1 + NUM_STATES_IN_ROW - 1) && state1 % NUM_STATES_IN_ROW != 0) {
			reachable = 0.1;
		} else if (action == ACTION_NORTH && state2 == (state1 + NUM_STATES_IN_ROW + 1) && state1 % NUM_STATES_IN_ROW != (NUM_STATES_IN_ROW - 1)) {
			reachable = 0.1;
		} else if (action == ACTION_SOUTH && state2 == (state1 - NUM_STATES_IN_ROW - 1) && state1 % NUM_STATES_IN_ROW != 0) {
			reachable = 0.1;
		} else if (action == ACTION_SOUTH && state2 == (state1 - NUM_STATES_IN_ROW + 1) && state1 % NUM_STATES_IN_ROW != (NUM_STATES_IN_ROW - 1)) {
			reachable = 0.1;
		} else if (action == ACTION_EAST && state2 == (state1 + NUM_STATES_IN_ROW + 1) && state1 % NUM_STATES_IN_ROW != (NUM_STATES_IN_ROW - 1)) {
			reachable = 0.1;
		} else if (action == ACTION_EAST && state2 == (state1 - NUM_STATES_IN_ROW + 1) && state1 % NUM_STATES_IN_ROW != (NUM_STATES_IN_ROW - 1)) {
			reachable = 0.1;
		} else if (action == ACTION_WEST && state2 == (state1 + NUM_STATES_IN_ROW - 1) && state1 % NUM_STATES_IN_ROW != 0) {
			reachable = 0.1;
		} else if (action == ACTION_WEST && state2 == (state1 - NUM_STATES_IN_ROW - 1) && state1 % NUM_STATES_IN_ROW != 0) {
			reachable = 0.1;
		} else if (action == ACTION_NORTHWEST && state2 == (state1 + NUM_STATES_IN_ROW)) {
			reachable = 0.1;
		} else if (action == ACTION_NORTHWEST && state2 == (state1 - 1) && state1 % NUM_STATES_IN_ROW != 0) {
			reachable = 0.1;
		} else if (action == ACTION_NORTHEAST && state2 == (state1 + 1) && state1 % NUM_STATES_IN_ROW != NUM_STATES_IN_ROW - 1) {
			reachable = 0.1;
		} else if (action == ACTION_NORTHEAST && state2 == (state1 + NUM_STATES_IN_ROW)) {
			reachable = 0.1;
		} else if (action == ACTION_SOUTHWEST && state2 == (state1 - 1) && state1 % NUM_STATES_IN_ROW != 0) {
			reachable = 0.1;
		} else if (action == ACTION_SOUTHWEST && state2 == (state1 - NUM_STATES_IN_ROW)) {
			reachable = 0.1;
		} else if (action == ACTION_SOUTHEAST && state2 == (state1 + 1) && state1 % NUM_STATES_IN_ROW != NUM_STATES_IN_ROW - 1) {
			reachable = 0.1;
		} else if (action == ACTION_SOUTHEAST && state2 == (state1 - NUM_STATES_IN_ROW)) {
			reachable = 0.1;
		} else if (state2 == state1) {
			if (state1 % NUM_STATES_IN_ROW == 0 && state1 != 0 && state1 != NUM_STATES - NUM_STATES_IN_ROW) {
				if (action == ACTION_WEST) {
					reachable = 1.0;
				} else if (action == ACTION_NORTHWEST) {
					reachable = 0.9;
				} else if (action == ACTION_SOUTHWEST) {
					reachable = 0.9;
				} else if (action == ACTION_EAST) {
					reachable = 0.0;
				} else if (action == ACTION_SOUTH) {
					reachable = 0.1;
				} else if (action == ACTION_NORTH) {
					reachable = 0.1;
				} else if (action == ACTION_SOUTHEAST) {
					reachable = 0.0;
				} else if (action == ACTION_NORTHEAST) {
					reachable = 0.0;
				}
			} else if (state1 == 0 || state1 == NUM_STATES_IN_ROW - 1 || state1 == NUM_STATES - NUM_STATES_IN_ROW || state1 == NUM_STATES -1) {
				if (state1 == 0) {
					if (action == ACTION_WEST) {
						reachable = 1.0;
					} else if (action == ACTION_NORTHWEST) {
						reachable = 0.9;
					} else if (action == ACTION_SOUTHWEST) {
						reachable = 1.0;
					} else if (action == ACTION_EAST) {
						reachable = 0.1;
					} else if (action == ACTION_SOUTH) {
						reachable = 1.0;
					} else if (action == ACTION_NORTH) {
						reachable = 0.1;
					} else if (action == ACTION_SOUTHEAST) {
						reachable = 0.9;
					} else if (action == ACTION_NORTHEAST) {
						reachable = 0.0;
					}
				} else if (state1 == NUM_STATES_IN_ROW -1) {
					if (action == ACTION_WEST) {
						reachable = 0.1;
					} else if (action == ACTION_NORTHWEST) {
						reachable = 0.0;
					} else if (action == ACTION_SOUTHWEST) {
						reachable = 0.9;
					} else if (action == ACTION_EAST) {
						reachable = 1.0;
					} else if (action == ACTION_SOUTH) {
						reachable = 1.0;
					} else if (action == ACTION_NORTH) {
						reachable = 0.1;
					} else if (action == ACTION_SOUTHEAST) {
						reachable = 1.0;
					} else if (action == ACTION_NORTHEAST) {
						reachable = 0.9;
					}
				} else if (state1 == NUM_STATES - NUM_STATES_IN_ROW) {
					if (action == ACTION_WEST) {
						reachable = 1.0;
					} else if (action == ACTION_NORTHWEST) {
						reachable = 1.0;
					} else if (action == ACTION_SOUTHWEST) {
						reachable = 0.9;
					} else if (action == ACTION_EAST) {
						reachable = 0.1;
					} else if (action == ACTION_SOUTH) {
						reachable = 0.1;
					} else if (action == ACTION_NORTH) {
						reachable = 1.0;
					} else if (action == ACTION_SOUTHEAST) {
						reachable = 0.0;
					} else if (action == ACTION_NORTHEAST) {
						reachable = 0.9;
					}
				} else if (state1 == NUM_STATES -1) {
					if (action == ACTION_WEST) {
						reachable = 0.1;
					} else if (action == ACTION_NORTHWEST) {
						reachable = 0.9;
					} else if (action == ACTION_SOUTHWEST) {
						reachable = 0.0;
					} else if (action == ACTION_EAST) {
						reachable = 1.0;
					} else if (action == ACTION_SOUTH) {
						reachable = 0.1;
					} else if (action == ACTION_NORTH) {
						reachable = 1.0;
					} else if (action == ACTION_SOUTHEAST) {
						reachable = 0.9;
					} else if (action == ACTION_NORTHEAST) {
						reachable = 1.0;
					}
				}
			} else if (state1 % NUM_STATES_IN_ROW == NUM_STATES_IN_ROW -1 && state1 != NUM_STATES -1 && state1 != NUM_STATES_IN_ROW -1) {
				if (action == ACTION_WEST) {
					reachable = 0.0;
				} else if (action == ACTION_NORTHWEST) {
					reachable = 0.0;
				} else if (action == ACTION_SOUTHWEST) {
					reachable = 0.0;
				} else if (action == ACTION_EAST) {
					reachable = 1.0;
				} else if (action == ACTION_SOUTH) {
					reachable = 0.1;
				} else if (action == ACTION_NORTH) {
					reachable = 0.1;
				} else if (action == ACTION_SOUTHEAST) {
					reachable = 0.9;
				} else if (action == ACTION_NORTHEAST) {
					reachable = 0.9;
				}
			} else if (state1 < NUM_STATES_IN_ROW && state1 != 0 && state1 != NUM_STATES_IN_ROW - 1) {
				if (action == ACTION_WEST) {
					reachable = 0.1;
				} else if (action == ACTION_NORTHWEST) {
					reachable = 0.0;
				} else if (action == ACTION_SOUTHWEST) {
					reachable = 0.9;
				} else if (action == ACTION_EAST) {
					reachable = 0.1;
				} else if (action == ACTION_SOUTH) {
					reachable = 1.0;
				} else if (action == ACTION_NORTH) {
					reachable = 0.0;
				} else if (action == ACTION_SOUTHEAST) {
					reachable = 0.9;
				} else if (action == ACTION_NORTHEAST) {
					reachable = 0.0;
				}
			} else if ((state1 +  NUM_STATES_IN_ROW) >= NUM_STATES && state1 != NUM_STATES - NUM_STATES_IN_ROW && state1 != NUM_STATES -1) {
				if (action == ACTION_WEST) {
					reachable = 0.1;
				} else if (action == ACTION_NORTHWEST) {
					reachable = 0.9;
				} else if (action == ACTION_SOUTHWEST) {
					reachable = 0.0;
				} else if (action == ACTION_EAST) {
					reachable = 0.1;
				} else if (action == ACTION_SOUTH) {
					reachable = 0.0;
				} else if (action == ACTION_NORTH) {
					reachable = 1.0;
				} else if (action == ACTION_SOUTHEAST) {
					reachable = 0.0;
				} else if (action == ACTION_NORTHEAST) {
					reachable = 0.9;
				}
			}
		}
		return reachable;
	}
	
	/*
	 * This function returns a reward for any state/action pair. It makes use of the transition
	 * function to determine what actions in what states lead to the goal state.
	 */
	public static double getRewardForGoal(int state, int action, double[][][] transitions, int goal_state) {
		//running into the wall is -100 reward. Doing anything in the goal state is 0
		if (state == goal_state) {
			return 0.0;
		} else {
			if (transitions[state][state][action] > 0.0) {
				return -100.0;
			}
			if (transitions[state][goal_state][action] > 0.0) {
				return 100.0;
			//transitioning anywhere else is -1 reward
			}
			return -1.0;
		}
	}
	
	/*
	 * This function returns a reward for any state/action pair. It makes use of the transition
	 * function to determine what actions in what states lead to the goal state.
	 */
	public static double getRewardForGoalWithObstacles(int state, int action, double[][][] transitions, int goal_state, Map<String,Integer> obstacles) {
		//running into the wall is -100 reward. Transitioning near an obstacle has a reward of -20. This gives us our buffer around obstacles 
				if (state == goal_state) {
					return 0.0;
				} else {
					for (Map.Entry<String, Integer> entry : obstacles.entrySet()) {
						if (state == entry.getValue()) {
							return -20.0;
						}
						if (transitions[state][entry.getValue()][action] > 0.0) {
							return -20.0;
						}
						int[] states = getSurroundingStates(entry.getValue());
						for (int i=0; i<NUM_ACTIONS; i++) {
							if(states[i] != -1) {
								if(transitions[state][states[i]][action] > 0.0) {
									return -20.0;
								}
								int[] sub_states = getSurroundingStates(states[i]);
									for (int j=0; j<NUM_ACTIONS; j++) {
										if (sub_states[j] != -1) {
											if (transitions[state][sub_states[j]][action] > 0.0) {
												return -20.0;
											}
										}
									}

							}
						}
					}
					if (transitions[state][state][action] > 0.0) {
						return -100.0;
					}
					if (transitions[state][goal_state][action] > 0.0) {
						return 1.0;
					//transitioning anywhere else is -1 reward
					}
					return -1.0;
				}
	}
	
	//Helper functions to get all transitions
	public static double[][][] getTransitions() {
		double[][][] trans = new double[MDPUtility.NUM_STATES][MDPUtility.NUM_STATES][MDPUtility.NUM_ACTIONS];
			for (int s=0; s<MDPUtility.NUM_STATES; s++) {
				for (int sprime=0; sprime<MDPUtility.NUM_STATES; sprime++) {
					for (int a=0; a<MDPUtility.NUM_ACTIONS; a++) {
						trans[s][sprime][a] = MDPUtility.getTransitionProbability(s, sprime, a);
					}
				}
			}
			return trans;
	}
	
	//Helper function to get all noisy transitions
	public static double[][][] getTransitionsNoisy() {
		double[][][] trans = new double[MDPUtility.NUM_STATES][MDPUtility.NUM_STATES][MDPUtility.NUM_ACTIONS];
			for (int s=0; s<MDPUtility.NUM_STATES; s++) {
				for (int sprime=0; sprime<MDPUtility.NUM_STATES; sprime++) {
					for (int a=0; a<MDPUtility.NUM_ACTIONS; a++) {
						trans[s][sprime][a] = MDPUtility.getTransitionProbabilityNoisy(s, sprime, a);
					}
				}
			}
			return trans;
	}
	
	//Helper function to get all rewards
	public static double[][] getRewards(double[][][] transitions, int goal_state) {
		double[][] rew = new double[MDPUtility.NUM_STATES][MDPUtility.NUM_ACTIONS];
		//Build reward model
		for (int s=0; s<MDPUtility.NUM_STATES; s++) {
			for (int a=0; a<MDPUtility.NUM_ACTIONS; a++) {
				rew[s][a] = MDPUtility.getRewardForGoal(s, a, transitions, goal_state);
			}
		}
		return rew;
	}
	
	//Helper function to get all rewards with obstacles
	public static double[][] getRewardsWithObstacles(double[][][] transitions, int goal_state, Map<String, Integer> obstacles) {
		double[][] rew = new double[MDPUtility.NUM_STATES][MDPUtility.NUM_ACTIONS];
		//Build reward model
		for (int s=0; s<MDPUtility.NUM_STATES; s++) {
			for (int a=0; a<MDPUtility.NUM_ACTIONS; a++) {
				rew[s][a] = MDPUtility.getRewardForGoalWithObstacles(s, a, transitions, goal_state, obstacles);
			}
		}
		return rew;
	}

	
	/*
	 * takes in a Q-table and produces a policy from it by selecting the action with the highest probability at each state 
	 */
	public static int[] generatePolicyFromQTable(double[][] q_table) {
		int[] policy = new int[NUM_STATES];
		for (int s = 0; s<NUM_STATES; s++) {
			double maxq = -Double.MAX_VALUE;
			int maxa = -1;
			for (int a = 0; a<NUM_ACTIONS; a++) {
				if (q_table[s][a] > maxq) {
					maxq = q_table[s][a];
					maxa = a;
				}
			}
			policy[s] = maxa;
		}
		return policy;
	}
	
	/*
	 * returns a single integer representing a state for any given, continuous x,y value. This is the tiling function
	 */
	public static int getStateForXandY(double x, double y) {
		int state = -1;
		int row = (int)y/TILE_SIZE;
		int column = (int)x/TILE_SIZE;
		state = row*NUM_STATES_IN_ROW + column;
		if (state > NUM_STATES) return NUM_STATES -1;
		if (state < 0) return 0;
		return state;
	}
}
