package uni.oldenburg.server.pathfinding;

import java.util.ArrayList;
import java.util.List;

import uni.oldenburg.server.pathfinding.GridItem.GridItemType;
import uni.oldenburg.shared.model.Point;

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
	
	protected boolean inArea(Point newPoint) {
		if (newPoint.getX() >= 0 && newPoint.getY() >= 0 && newPoint.getX() < myColumnCount && newPoint.getY() < myRowCount)
			return true;
		
		return false;
	}
	
	public static int getIndex(int x, int y, int myColumnCount) {
		return y * myColumnCount + x;
	}
	
	public static int getIndex(Point newPoint, int myColumnCount){
		return getIndex(newPoint.getX(), newPoint.getY(), myColumnCount);
	}
	
	protected boolean isWall(Point newPoint, Direction newDirection) {
		if (!inArea(newPoint)) 
			return true;
		
		int index = getIndex(getNeighborPoint(newPoint, newDirection), myColumnCount);
		
		if (index >= lstGridItem.size() || index < 0)
			return true;
		
		GridItem neighborItem = lstGridItem.get(index);
		
		if(neighborItem.getItemType() == GridItemType.WallItem)
			return true;
		
		return false;
	}
	
	protected GridValueReturnType setValueOfSurroundingBlock(GridItem curItem, Point curPoint, Direction newDirection, List<Point> lstBlocksWithNewValues) {
		Point neighborPoint = getNeighborPoint(curPoint, newDirection);
		int nextStepValue = 10;
		
		if (!inArea(neighborPoint))
			return GridValueReturnType.DoesNotExists;
		
		GridItem nextItem = lstGridItem.get(getIndex(neighborPoint.getX(), neighborPoint.getY(), myColumnCount));
		
		if (nextItem.getItemType() != GridItemType.WallItem){
			if (newDirection == Direction.TopLeft ||
				newDirection == Direction.TopRight ||
				newDirection == Direction.BottomLeft ||
				newDirection == Direction.BottomRight) {
				
				nextStepValue= 14;
			}
			
			if (nextItem.getGridValue() < 0 || nextItem.getGridValue() > curItem.getGridValue() + nextStepValue){
				nextItem.setGridValue(nextStepValue + curItem.getGridValue());
				
				lstBlocksWithNewValues.add(neighborPoint);
		
				return GridValueReturnType.ValueUpdated;
			}
		}
		else 
			return GridValueReturnType.IsWall;
		
		return GridValueReturnType.Unchanged;
	}
	
	/*protected void ComputeBlocksValues(){
		List<Point> lstBlocksWithNewValue = new ArrayList<Point>();
		
		
	}*/
}