package mdp;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
public class ValueIterationDriver {
	public static final boolean OUTPUT_TRANSITIONS = false;
	public static final boolean OUTPUT_REWARDS = false;
	public static final boolean OUTPUT_POLICY = false;
	public static final boolean OUTPUT_LOADABLE_POLICY = true;
	public static final String OUTPUT_DIRECTORY = "/Users/collinst/Desktop/";

	/**
	 * @param args
	 * @throws IOException 
	 */
	public static void main(String[] args) throws IOException {
		FileWriter fstream, fstream2, fstream3, fstream4;
		BufferedWriter mout, mout2, mout3, mout4;
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
		//Build transition model
		double[][][] trans = MDPUtility.getTransitions();
		//Build reward model
		double[][] rew = MDPUtility.getRewards(trans, 0);
		
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
	}

}

