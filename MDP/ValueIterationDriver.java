package mdp;
import java.io.BufferedWriter;
import java.util.*;
import java.io.FileWriter;
import java.io.IOException;
public class ValueIterationDriver {
	public static final boolean OUTPUT_TRANSITIONS = false;
	public static final boolean OUTPUT_REWARDS = false;
	public static final boolean OUTPUT_POLICY = false;
	public static final boolean OUTPUT_LOADABLE_POLICY = true;
	public static final boolean OUTPUT_LOADABLE_POLICY_UPDATED = true;
	public static final boolean OUTPUT_REWARDS_UPDATED = false;
	public static final String OUTPUT_DIRECTORY = "/Users/collinst/Desktop/";

	/**
	 * @param args
	 * @throws IOException 
	 */
	public static void main(String[] args) throws IOException {
		
		FileWriter fstream, fstream2, fstream3, fstream4, fstream5, fstream6;
		BufferedWriter mout, mout2, mout3, mout4, mout5, mout6;
		if (OUTPUT_TRANSITIONS) {
			fstream = new FileWriter(OUTPUT_DIRECTORY + "transitions.txt");
		}
		if (OUTPUT_REWARDS) {
			fstream2 = new FileWriter(OUTPUT_DIRECTORY + "rewards.txt");
		}
		if (OUTPUT_POLICY) {
			fstream3 = new FileWriter(OUTPUT_DIRECTORY + "policy.txt");
		}
		if (OUTPUT_LOADABLE_POLICY) {
			fstream4 = new FileWriter(OUTPUT_DIRECTORY + "policy_load.txt");
		}
		if (OUTPUT_LOADABLE_POLICY_UPDATED) {
			fstream5 = new FileWriter(OUTPUT_DIRECTORY + "policy_load_updated.txt");
		}
		if (OUTPUT_REWARDS_UPDATED) {
			fstream6 = new FileWriter(OUTPUT_DIRECTORY + "rewards_updated.txt");
		}
		if (OUTPUT_TRANSITIONS) {
			mout = new BufferedWriter(fstream);
		}
		if (OUTPUT_REWARDS) {
			mout2 = new BufferedWriter(fstream2);
		}
		if (OUTPUT_POLICY) {
			mout3 = new BufferedWriter(fstream3);
		}
		if (OUTPUT_LOADABLE_POLICY) {
			mout4 = new BufferedWriter(fstream4);
		}
		if (OUTPUT_LOADABLE_POLICY_UPDATED) {
			mout5= new BufferedWriter(fstream5);
		}
		if (OUTPUT_REWARDS_UPDATED) {
			mout6 = new BufferedWriter(fstream6);
		}
		double[][][] trans = MDPUtility.getTransitions();
		//Get rewards for the given goal state
		Map<String, Integer> m = new HashMap<String, Integer>();
		m.put("1", 310);
		m.put("2", 67);
		double[][] rew = MDPUtility.getRewardsWithObstacles(trans, 190, m);
		
		if (OUTPUT_TRANSITIONS) {
			for (int i=0;i<MDPUtility.NUM_STATES; i++) {
				for (int j=0; j<MDPUtility.NUM_STATES; j++) {
					mout.write("From state: " + i + " to state: " + j + " P = " + trans[i][j][MDPUtility.ACTION_NORTH] + " via action NORTH\n");
				}
			}
		}
		
		if (OUTPUT_REWARDS) {
			for (int i=0;i<MDPUtility.NUM_STATES; i++) {
				for (int j=0; j<MDPUtility.NUM_ACTIONS; j++) {
					String action_string = "";
					if (j == MDPUtility.ACTION_NORTH) {
						action_string = "North";
					} else if (j == MDPUtility.ACTION_SOUTH) {
						action_string = "South";
					} else if (j == MDPUtility.ACTION_EAST) {
						action_string = "East";
					} else if (j == MDPUtility.ACTION_WEST) {
						action_string = "West";
					} else if (j == MDPUtility.ACTION_NORTHWEST) {
						action_string = "Northwest";
					} else if (j == MDPUtility.ACTION_NORTHEAST) {
						action_string = "Northeast";
					} else if (j == MDPUtility.ACTION_SOUTHWEST) {
						action_string = "Southwest";
					} else if (j == MDPUtility.ACTION_SOUTHEAST) {
						action_string = "Southeast";
					}
					mout2.write("Taking action " + action_string + " in state " + i + " gives reward " + rew[i][j] + "\n");
				}
			}	
		}
		
		double[][] q_table = MDPUtility.valueIteration(trans, rew);
		int[] policy = MDPUtility.generatePolicyFromQTable(q_table);
		if (OUTPUT_POLICY || OUTPUT_LOADABLE_POLICY) {
			for (int s=0;s<MDPUtility.NUM_STATES; s++) {
				String action_string = "";
				if (policy[s] == MDPUtility.ACTION_NORTH) {
					action_string = "North";
				} else if (policy[s] == MDPUtility.ACTION_SOUTH) {
					action_string = "South";
				} else if (policy[s] == MDPUtility.ACTION_EAST) {
					action_string = "East";
				} else if (policy[s] == MDPUtility.ACTION_WEST) {
					action_string = "West";
				} else if (policy[s] == MDPUtility.ACTION_NORTHWEST) {
					action_string = "Northwest";
				} else if (policy[s] == MDPUtility.ACTION_NORTHEAST) {
					action_string = "Northeast";
				} else if (policy[s] == MDPUtility.ACTION_SOUTHWEST) {
					action_string = "Southwest";
				} else if (policy[s] == MDPUtility.ACTION_SOUTHEAST) {
					action_string = "Southeast";
				}
				if (OUTPUT_POLICY) {
					mout3.write("Take action " + action_string + " in state " + s + "\n");
				}
				if (OUTPUT_LOADABLE_POLICY) {
					mout4.write(policy[s] + ",");
				}
			}
		}
		
		rew = MDPUtility.updateRewardsRealTime(193, 190, trans, rew);
		q_table = MDPUtility.valueIterationRealTime(193, 190, trans, rew, q_table);
		policy = MDPUtility.updatePolicyRealTime(policy, q_table, 193, 190);
		
		for (int s=0; s<MDPUtility.NUM_STATES; s++) {
			String action_string = "";
			if (policy[s] == MDPUtility.ACTION_NORTH) {
				action_string = "North";
			} else if (policy[s] == MDPUtility.ACTION_SOUTH) {
				action_string = "South";
			} else if (policy[s] == MDPUtility.ACTION_EAST) {
				action_string = "East";
			} else if (policy[s] == MDPUtility.ACTION_WEST) {
				action_string = "West";
			} else if (policy[s] == MDPUtility.ACTION_NORTHWEST) {
				action_string = "Northwest";
			} else if (policy[s] == MDPUtility.ACTION_NORTHEAST) {
				action_string = "Northeast";
			} else if (policy[s] == MDPUtility.ACTION_SOUTHWEST) {
				action_string = "Southwest";
			} else if (policy[s] == MDPUtility.ACTION_SOUTHEAST) {
				action_string = "Southeast";
			}
			if (OUTPUT_POLICY) {
				mout3.write("Take action " + action_string + " in state " + s + "\n");
			}
			if (OUTPUT_LOADABLE_POLICY_UPDATED) {
				mout5.write(policy[s] + ",");
			}
		}
		
		if (OUTPUT_REWARDS_UPDATED) {
			for (int i=0;i<MDPUtility.NUM_STATES; i++) {
				for (int j=0; j<MDPUtility.NUM_ACTIONS; j++) {
					String action_string = "";
					if (j == MDPUtility.ACTION_NORTH) {
						action_string = "North";
					} else if (j == MDPUtility.ACTION_SOUTH) {
						action_string = "South";
					} else if (j == MDPUtility.ACTION_EAST) {
						action_string = "East";
					} else if (j == MDPUtility.ACTION_WEST) {
						action_string = "West";
					} else if (j == MDPUtility.ACTION_NORTHWEST) {
						action_string = "Northwest";
					} else if (j == MDPUtility.ACTION_NORTHEAST) {
						action_string = "Northeast";
					} else if (j == MDPUtility.ACTION_SOUTHWEST) {
						action_string = "Southwest";
					} else if (j == MDPUtility.ACTION_SOUTHEAST) {
						action_string = "Southeast";
					}
					mout6.write("Taking action " + action_string + " in state " + i + " gives reward " + rew[i][j] + "\n");
				}
			}	
		}
		
		if (OUTPUT_TRANSITIONS) {
			mout.close();
		}
		if (OUTPUT_REWARDS) {
			mout2.close();
		}
		if (OUTPUT_POLICY) {
			mout3.close();
		}
		if (OUTPUT_LOADABLE_POLICY) {
			mout4.close();
		}
		if (OUTPUT_LOADABLE_POLICY_UPDATED) {
			mout5.close();
		}
		if (OUTPUT_REWARDS_UPDATED) {
			mout6.close();
		}
		
	}

}

