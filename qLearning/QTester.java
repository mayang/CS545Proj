package qLearning;

public class QTester {
	
	public static final int NUM_EPISODES = 20000000;
	
	public static void main(String[] args) throws Exception {
		final long startTime = System.currentTimeMillis();
		final long endTime;
		int[] policy;
		try {
			System.out.println(QUtilitiesObstacle2.XYtoState(200, 45));
			System.out.println(QUtilitiesObstacle2.OBSTACLE_STATES);
		
			policy = QUtilitiesObstacle2.QtoPolicy(QUtilitiesObstacle2.generateQTable(2, QUtilitiesObstacle2.XYtoState(450, 450)));
		
		} finally {
			  endTime = System.currentTimeMillis();
			}
			final long duration = endTime - startTime;
			System.out.println(duration);
			for(int i =0 ; i < policy.length; i++){
				System.out.print(policy[i] + ",");
			}
	}
}