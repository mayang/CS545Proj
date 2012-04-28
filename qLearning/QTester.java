package qLearning;

public class QTester {
	
	public static final int NUM_EPISODES = 100000;
	
	public static void main(String[] args) throws Exception {
		final long startTime = System.currentTimeMillis();
		final long endTime;
		int[] policy;
		try {
//			policy = QUtilities.QtoPolicy(QUtilities.generateQTable(1, 599));
			QUtilitiesObstacle2.setObstacleStatesAbsolute(new int[] {200,10,200,30,200,50,200,70,200,90,200,110,200,130,200,150,200,170,200,190});
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