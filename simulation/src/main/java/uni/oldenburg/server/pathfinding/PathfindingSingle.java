package uni.oldenburg.server.pathfinding;

import java.util.ArrayList;
import java.util.List;

import uni.oldenburg.server.pathfinding.GridItem.GridItemType;
import uni.oldenburg.shared.model.Point;

public class PathfindingSingle extends Pathfinding {
	List<List<PathPoint>> lstPathPoints = new ArrayList<List<PathPoint>>();
	
	public PathfindingSingle() {};
	
	public PathfindingSingle(int columnCount, int rowCount, List<GridItem> lstGridItem) {
		init(columnCount, rowCount, lstGridItem);
	};
	
	
	@Override
	public PathMessageType findPath(Point newStartPoint, Point newStopPoint, List<List<PathPoint>> lstPathPoints) {
		List<PathPoint> lstSinglePathpoints = new ArrayList<PathPoint>();
		PathMessageType returnValue = PathMessageType.Running;
		
		if (bRunning)
			return returnValue;
		
		lstPathPoints.clear();
		
		myStartPoint = newStartPoint;
		myStopPoint = newStopPoint;
		
		if(myStartPoint.getX() < 0 || myStartPoint.getY() < 0)
			return PathMessageType.PathError;
		
		if(myStopPoint.getX() < 0 || myStopPoint.getY() < 0)
			return PathMessageType.PathError;		
		
		if (lstGridItem == null)
			return PathMessageType.NotInitialized;
		
		lstGridItem.get((getIndex(myStartPoint, myColumnCount))).setItemType(GridItemType.StartItem);
        lstGridItem.get((getIndex(myStopPoint, myColumnCount))).setItemType(GridItemType.StopItem);
		
        if(myStartPoint.getX() == myStopPoint.getX() && myStartPoint.getY() == myStopPoint.getY())
        	return PathMessageType.StartEndPointsEqual;
        
        bRunning = true;
        
        ComputeBlocksValues();
        
        PathPoint curPathPoint= new PathPoint();
        curPathPoint.setPoint(new Point(myStopPoint.getX(), myStopPoint.getY()));
        
        curPathPoint.Direction = Direction.Left;
        
        List<PathPoint> lstPossiblePoints = new ArrayList<PathPoint>();
        
        PathPoint stopPathPoint = new PathPoint();
        stopPathPoint.setPoint(myStopPoint);
        lstSinglePathpoints.add(stopPathPoint);
        
        returnValue = PathMessageType.PathFound;
        
        while(curPathPoint.getPoint().getX() != myStartPoint.getX() || curPathPoint.getPoint().getY() != myStartPoint.getY()){
        	curPathPoint.setStepValue(lstGridItem.get(getIndex(curPathPoint.getPoint().getX(), curPathPoint.getPoint().getY(), myColumnCount)).getGridValue());
            int minValue = curPathPoint.getStepValue();

            saveBestPoint(curPathPoint, Direction.Left, minValue, lstPossiblePoints);
            saveBestPoint(curPathPoint, Direction.Top, minValue, lstPossiblePoints);
            saveBestPoint(curPathPoint, Direction.Right, minValue, lstPossiblePoints);
            saveBestPoint(curPathPoint, Direction.Bottom, minValue, lstPossiblePoints);
    	            
            if(bDriveDiagonal){
            	saveBestPoint(curPathPoint, Direction.TopLeft, minValue, lstPossiblePoints);
                saveBestPoint(curPathPoint, Direction.TopRight, minValue, lstPossiblePoints);
                saveBestPoint(curPathPoint, Direction.BottomLeft, minValue, lstPossiblePoints);
                saveBestPoint(curPathPoint, Direction.BottomRight, minValue, lstPossiblePoints);
            	
            }
            
        }
        
        
        
		return null;
	}
}
