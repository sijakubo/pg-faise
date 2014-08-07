package uni.oldenburg.server.pathfinding;

import java.util.ArrayList;
import java.util.List;

import uni.oldenburg.server.pathfinding.GridItem.GridItemType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.Point;
/**
 * Pathfinding implementiert IPathfinding. Stellt Basisfunktionalitäten für den implementieren Pathfindingalgorithmus bereit.
 * 
 * @author Matthias
 */	
public abstract class Pathfinding implements IPathfinding {
	public enum PathMessageType {//Enum dient dazu den Status und eventuelle Fehler des Pathfindings zu kodieren
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
	
	protected int myMinValue = 0;
	
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
				/*switch (lstGridItem.get(Pathfinding.getIndex2(x, y, myColumnCount)).getItemType()) {
					case WallItem:
					case DefaultItem:
					case PathItem:
					case StartItem:
					case StopItem:
						System.out.print("[" + lstGridItem.get(Pathfinding.getIndex2(x, y, myColumnCount)).getGridValue() + "]");
						break;
					default:
						break;
				}*/
				
				switch (lstGridItem.get(Pathfinding.getIndex2(x, y, myColumnCount)).getItemType()) {
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
				}
			}
			System.out.println("");
		}
	}
	
	protected Point getNeighborPoint(Point curPoint, Direction newDirection) {
		Point tmpPoint = null;
		
		switch (newDirection){
		case Left:
			tmpPoint = new Point (curPoint.getX() - Conveyor.RASTER_SIZE, curPoint.getY());
			break;
		case Top:
			tmpPoint = new Point (curPoint.getX(), curPoint.getY() - Conveyor.RASTER_SIZE);
			break;
		case Right:
			tmpPoint = new Point (curPoint.getX() + Conveyor.RASTER_SIZE, curPoint.getY());
			break;
		case Bottom:
			tmpPoint = new Point (curPoint.getX(), curPoint.getY() + Conveyor.RASTER_SIZE);
			break;
		case TopLeft:
			tmpPoint = new Point (curPoint.getX() - Conveyor.RASTER_SIZE, curPoint.getY() - Conveyor.RASTER_SIZE);
			break;
		case TopRight:
			tmpPoint = new Point (curPoint.getX() + Conveyor.RASTER_SIZE, curPoint.getY() - Conveyor.RASTER_SIZE);
			break;
		case BottomLeft:
			tmpPoint = new Point (curPoint.getX() - Conveyor.RASTER_SIZE, curPoint.getY() + Conveyor.RASTER_SIZE);
			break;
		case BottomRight:
			tmpPoint = new Point (curPoint.getX() + Conveyor.RASTER_SIZE, curPoint.getY() + Conveyor.RASTER_SIZE);
			break;		
		
		}
		
		return tmpPoint;
	}
	
	protected boolean inArea(Point newPoint) {
		if (newPoint.getX() >= 0 && newPoint.getY() >= 0 && newPoint.getX() < (myColumnCount * Conveyor.RASTER_SIZE) && newPoint.getY() < (myRowCount * Conveyor.RASTER_SIZE))
			return true;
		
		return false;
	}
	
	public static int getIndex(int x, int y, int myColumnCount) {
		return (y / Conveyor.RASTER_SIZE) * myColumnCount + (x / Conveyor.RASTER_SIZE);
	}
	
	public static int getIndex2(int x, int y, int myColumnCount) {
		return (y) * myColumnCount + (x);
	}
	
	public static int getIndex2(Point newPoint, int myColumnCount){
		return getIndex2(newPoint.getX(), newPoint.getY(), myColumnCount);
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
		
		//System.out.println("c: " + curPoint.toString());
		//System.out.println("d: " + neighborPoint.toString());
		
		if (!inArea(neighborPoint))
			return GridValueReturnType.DoesNotExists;
		
		GridItem nextItem = lstGridItem.get(getIndex(neighborPoint.getX(), neighborPoint.getY(), myColumnCount));
		
		if (nextItem.getItemType() != GridItemType.WallItem) {
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
	
	protected void resetValues(){
		for (GridItem myItem : lstGridItem){
			myItem.setItemType(myItem.getItemType(), true);
		}
		
	}
	
	protected void ComputeBlocksValues(){
		List<Point> lstBlocksWithNewValue = new ArrayList<Point>();
		
		List<Direction> lstDirectionBase =  new ArrayList<Direction>();
		lstDirectionBase.add(Direction.Left);
		lstDirectionBase.add(Direction.Top);
		lstDirectionBase.add(Direction.Right);
		lstDirectionBase.add(Direction.Bottom);
		
		lstBlocksWithNewValue.add(new Point(myStartPoint.getX(), myStartPoint.getY()));
		
		resetValues();
		
		while (lstBlocksWithNewValue.size() > 0){
			Point curPoint = lstBlocksWithNewValue.get(0);
			lstBlocksWithNewValue.remove(curPoint);
			
			GridItem myItem = lstGridItem.get(getIndex(curPoint.getX(), curPoint.getY(), myColumnCount));
			
			//System.out.println(myItem.myGridValue);
			
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
	
	protected List<PathPoint> saveBestPoint(PathPoint curPathPoint, Direction newDirection, List<PathPoint> lstPossiblePoints){
		Point neighborPoint = getNeighborPoint(curPathPoint.getPoint(), newDirection);
		int newValue = getGridValue(neighborPoint);	
		
		switch(newDirection){
		case TopLeft:
			 if (isWall(curPathPoint.getPoint(), Direction.Top)) return lstPossiblePoints;
			 if (isWall(curPathPoint.getPoint(), Direction.Left)) return lstPossiblePoints;
			break;
		case TopRight:
			if (isWall(curPathPoint.getPoint(), Direction.Top)) return lstPossiblePoints;
			if (isWall(curPathPoint.getPoint(), Direction.Right)) return lstPossiblePoints;
			break;
		case BottomLeft:
			if (isWall(curPathPoint.getPoint(), Direction.Bottom)) return lstPossiblePoints;
			if (isWall(curPathPoint.getPoint(), Direction.Left)) return lstPossiblePoints;
			break;
		case BottomRight:
			if (isWall(curPathPoint.getPoint(), Direction.Bottom)) return lstPossiblePoints;
			if (isWall(curPathPoint.getPoint(), Direction.Right)) return lstPossiblePoints;
			 break;
		default:
			break;
		}
		
		if (/*newValue <= curPathPoint.getStepValue() &&*/ newValue <= myMinValue) {
			//System.out.println("better value found");
			
			if (bDriveDiagonal){
				List<PathPoint> lstDeleteableValues= new ArrayList<PathPoint>();
				
				for (PathPoint myPoint : lstPossiblePoints){
					if (getGridValue(myPoint.getPoint()) > newValue){
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
			
			myMinValue = newValue;
		}
		
		return lstPossiblePoints;
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