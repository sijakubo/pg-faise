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

	public void init(int columnCount, int rowCount, List<GridItem> lstGridItem) {
		myColumnCount = columnCount;
		myRowCount = rowCount;
		
		this.lstGridItem = lstGridItem;
	}

	public abstract List<List<PathPoint>> findPath(Point newStartPoint, Point newStopPoint);
	
	public static void drawGrid(int myColumnCount, int myRowCount, List<GridItem> lstGridItem) {
		for (int y = 0; y < myRowCount; ++y) {
			for (int x = 0; x < myColumnCount; ++x) {
				switch (lstGridItem.get(Pathfinding.getIndex(x, y, myColumnCount)).getItemType()) {
					case WallItem:
					case DefaultItem:
					case PathItem:
					case StartItem:
					case StopItem:
						System.out.print("[" + lstGridItem.get(Pathfinding.getIndex(x, y, myColumnCount)).getGridValue() + "]");
						break;
					default:
						break;
				}
				
				/*switch (lstGridItem.get(Pathfinding.getIndex(x, y, myColumnCount)).getItemType()) {
					case WallItem:
						System.out.print("[x]");
						break;
					case DefaultItem:
						System.out.print("[ ]");
						break;
					case PathItem:
						System.out.print("[.]");
						break;
					case StartItem:
						System.out.print("[a]");
						break;
					case StopItem:
						System.out.print("[e]");
						break;
					default:
						break;
				}*/
			}
			System.out.println("");
		}
	}
	
	
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
		
		System.out.println("isWall: " + (nextItem.getItemType() == GridItemType.WallItem));
		//System.out.println("isNotWall: " + (nextItem.getItemType() != GridItemType.WallItem));
		
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
	
	protected void resetValues() {
		for (GridItem myItem : lstGridItem) {
			if (!myItem.getItemType().equals(GridItemType.WallItem)) {
				myItem.setItemType(GridItemType.DefaultItem);
			}
			else {
				myItem.setItemType(GridItemType.WallItem);
			}
		}
		
		/*for (GridItem myItem : lstGridItem){
			myItem.setItemType(myItem.getItemType(), true);
		}*/
		
	}
	
	protected void ComputeBlocksValues(){
		List<Point> lstBlocksWithNewValue = new ArrayList<Point>();
		
		List<Direction> lstDirectionBase =  new ArrayList<Direction>();
		lstDirectionBase.add(Direction.Left);
		lstDirectionBase.add(Direction.Top);
		lstDirectionBase.add(Direction.Right);
		lstDirectionBase.add(Direction.Bottom);
		
		lstBlocksWithNewValue.add(new Point(myStartPoint.getX(), myStartPoint.getY()));
		
		while (lstBlocksWithNewValue.size() > 0){
			Point curPoint = lstBlocksWithNewValue.get(0);
			lstBlocksWithNewValue.remove(curPoint);
			
			GridItem myItem = lstGridItem.get(getIndex(curPoint.getX(), curPoint.getY(), myColumnCount));
			
			if (myItem.myGridValue >= 0) {
				for (Direction myDirection : lstDirectionBase) {
					setValueOfSurroundingBlock(myItem, curPoint, myDirection, lstBlocksWithNewValue);
				}
			
			
				if(bDriveDiagonal){
					setValueOfSurroundingBlock(myItem, curPoint, Direction.TopLeft, lstBlocksWithNewValue);
					setValueOfSurroundingBlock(myItem, curPoint, Direction.TopRight, lstBlocksWithNewValue);
					setValueOfSurroundingBlock(myItem, curPoint, Direction.BottomLeft, lstBlocksWithNewValue);
					setValueOfSurroundingBlock(myItem, curPoint, Direction.BottomRight, lstBlocksWithNewValue);
				}
			}
			
		}
	}
	
	protected int getGridValue(Point newPoint){
		return getGridValue(newPoint.getX(), newPoint.getY());
		
	}
	
	protected int getGridValue(int x, int y){
		if (inArea(new Point(x,y))){
			int value = lstGridItem.get(getIndex(new Point(x, y), myColumnCount)).getGridValue();
			
			if (value >= 0){
				return value;
			}
			
		}
		return lstGridItem.get(getIndex(myStopPoint.getX(), myStartPoint.getY(), myColumnCount)).getGridValue()+1;

	}
	
	protected int saveBestPoint(PathPoint curPathPoint, Direction newDirection, int minValue,List<PathPoint> lstPossiblePoints){
		Point neighborPoint = getNeighborPoint(curPathPoint.getPoint(), newDirection);
		int newValue = getGridValue(neighborPoint);	
		
		
		switch(newDirection){
		case TopLeft:
			 if (isWall(curPathPoint.getPoint(), Direction.Top)) return minValue;
			 if (isWall(curPathPoint.getPoint(), Direction.Left)) return minValue;
			break;
		case TopRight:
			if (isWall(curPathPoint.getPoint(), Direction.Top)) return minValue;
			if (isWall(curPathPoint.getPoint(), Direction.Right)) return minValue;
			break;
		case BottomLeft:
			if (isWall(curPathPoint.getPoint(), Direction.Bottom)) return minValue;
			if (isWall(curPathPoint.getPoint(), Direction.Left)) return minValue;
			break;
		case BottomRight:
			if (isWall(curPathPoint.getPoint(), Direction.Bottom)) return minValue;
			if (isWall(curPathPoint.getPoint(), Direction.Right)) return minValue;
			 break;
		default:
			break;
		
		}
		
		if (newValue <= curPathPoint.getStepValue() && newValue <= minValue) {
			if (bDriveDiagonal){
				List<PathPoint> lstDeleteableValues = new ArrayList<PathPoint>();
				
				for (PathPoint myPoint : lstPossiblePoints){
					if (getGridValue(myPoint.getPoint()) > newValue) {
						lstDeleteableValues.add(myPoint);
					}
				}
				
				for (PathPoint myPoint : lstDeleteableValues){
					lstPossiblePoints.remove(myPoint);
				}
				
				lstDeleteableValues.clear();
				
			}
			
			PathPoint myPathPoint = new PathPoint();
			myPathPoint.setPoint(neighborPoint);
			myPathPoint.setStepValue(newValue);
			myPathPoint.setEstimationValue(myPathPoint.getStepValue());
			myPathPoint.Direction = newDirection;
			
			if (curPathPoint.Direction != newDirection){
				myPathPoint.setEstimationValue(myPathPoint.getEstimationValue() + myPathPoint.getEstimationValue()); 				
			}
			
			lstPossiblePoints.add(myPathPoint);
			
			minValue = newValue;
		}
		
		return minValue;
	}

	
	public static int countRotations(List<PathPoint> lstPathPoints){
		
		int numRotation = 0;
		
		if(lstPathPoints.size() == 0){
			return numRotation;
		}
		
		Direction lastDirection = lstPathPoints.get(0).Direction;
		
		for (PathPoint myPathPoint : lstPathPoints){
			if (lastDirection != myPathPoint.Direction){
				numRotation += 1;
			}
			
			lastDirection = myPathPoint.Direction;
		}
		
		return numRotation;
		
	}
	// später prüfen
	public static int countRotations2(List<List<PathPoint>> lstPathPoint) {
		if(lstPathPoint.size() >=1){
			return countRotations(lstPathPoint.get(0));
			
		}
		return 0;
	}
	
	
	
	
}