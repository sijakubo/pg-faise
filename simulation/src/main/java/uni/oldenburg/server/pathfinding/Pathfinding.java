package uni.oldenburg.server.pathfinding;

import java.util.List;

import com.google.gwt.touch.client.Point;

public abstract class Pathfinding implements IPathfinding {
	public enum PathMessageType {
		PathFound,
		PathBlocked,
		PathError,
		Running,
		StartEndPointsEqual,
		NotInitialized
	}
	
	public enum Direction {
		Left,
        Top,
        Right,
        Bottom,

        TopLeft,
        TopRight,
        BottomLeft,
        BottomRight
	}
	
    protected enum GridValueReturnType {
        DoesNotExists,
        ValueUpdated,
        Unchanged,
        IsWall
	}
	
	protected List<GridItem> lstGridItem = null;
	protected boolean bRunning = false;
	protected int myColumnCount = 0;
	protected int myRowCount = 0;
	
	protected Point myStartPoint = new Point (-1, -1);
	protected Point myStopPoint = new Point (-1, -1);
	
	protected boolean bDriveDiagonal = false;
	protected boolean bAvoidRotations = false;
	
	public int getColumnCount() {
		return myColumnCount;
	}

	public boolean getDriveDiagonal() {
		return bDriveDiagonal;
	}

	public void setDriveDiagonal(boolean value) {
		bDriveDiagonal = value;
	}

	public boolean getAvoidRotations() {
		return bAvoidRotations;
	}

	public void setAvoidRotations(boolean value) {
		bAvoidRotations = value;
	}

	public boolean isRunning() {
		return bRunning;
	}

	public int getRowCount() {
		return myRowCount;
	}

	public Point getStartPoint() {
		return myStartPoint;
	}

	public Point getStopPoint() {
		return myStopPoint;
	}

	public void Init(int columnCount, int rowCount, List<GridItem> lstGridItem) {
		myColumnCount = columnCount;
		myRowCount = rowCount;
		
		this.lstGridItem = lstGridItem;
	}

	public abstract PathMessageType findPath(Point newStartPoint, Point newStopPoint, List<List<PathPoint>> lstPathPoints);
	
	protected Point getNeighborPoint(Point curPoint, Direction newDirection) {
		Point tmpPoint = null;
		
		switch (newDirection){
		case Left:
			tmpPoint = new Point (curPoint.getX() - 1, curPoint.getY());
			break;
		case Top:
			tmpPoint = new Point (curPoint.getX(), curPoint.getY() - 1);
			break;
		case Right:
			tmpPoint = new Point (curPoint.getX() + 1, curPoint.getY());
			break;
		case Bottom:
			tmpPoint = new Point (curPoint.getX(), curPoint.getY() + 1);
			break;
		case TopLeft:
			tmpPoint = new Point (curPoint.getX() - 1, curPoint.getY() - 1);
			break;
		case TopRight:
			tmpPoint = new Point (curPoint.getX() + 1, curPoint.getY() - 1);
			break;
		case BottomLeft:
			tmpPoint = new Point (curPoint.getX() - 1, curPoint.getY() + 1);
			break;
		case BottomRight:
			tmpPoint = new Point (curPoint.getX() + 1, curPoint.getY() + 1);
			break;		
		
		}
		
		return tmpPoint;
	}
	
}