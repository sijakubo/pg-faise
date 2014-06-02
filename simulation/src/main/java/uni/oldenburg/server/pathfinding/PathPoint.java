package uni.oldenburg.server.pathfinding;

import java.awt.Point;

public class PathPoint {
	private Point myPoint= new Point(0,0);
	private int myStepValue = 0;
	private int myEstimationValue = 0;
	
	uni.oldenburg.server.pathfinding.Pathfinding.Direction Direction;
	
	public Point getPoint() {
		return myPoint;
	}
	public void setPoint(Point myPoint) {
		this.myPoint = myPoint;
	}
	public int getStepValue() {
		return myStepValue;
	}
	public void setStepValue(int myStepValue) {
		this.myStepValue = myStepValue;
	}
	public int getEstimationValue() {
		return myEstimationValue;
	}
	public void setEstimationValue(int myEstimationValue) {
		this.myEstimationValue = myEstimationValue;
	}
	
	
	

}
