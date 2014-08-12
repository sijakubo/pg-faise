package uni.oldenburg.server.pathfinding;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import uni.oldenburg.server.pathfinding.GridItem.GridItemType;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.Point;

public class PathfindingSingle extends Pathfinding {
	List<List<PathPoint>> lstPathPoints = new ArrayList<List<PathPoint>>();
	
	PathMessageType returnValue = PathMessageType.Running;
	
	public PathfindingSingle() {};
	
	public PathfindingSingle(int columnCount, int rowCount, List<GridItem> lstGridItem) {
		init(columnCount, rowCount, lstGridItem);
	};
	
	public PathMessageType getStatus() {
		return returnValue;
	}
	
	@Override
	public List<List<PathPoint>> findPath(Point newStartPoint, Point newStopPoint) {
		List<PathPoint> lstSinglePathpoints = new ArrayList<PathPoint>();
		returnValue = PathMessageType.Running;
		
		if (bRunning)
			return null;
		
		if (lstPathPoints != null)
			lstPathPoints.clear();
		else {
			lstPathPoints = new ArrayList<List<PathPoint>>();
		}
		
		myStartPoint = new Point(newStartPoint.getX() / Conveyor.RASTER_SIZE, newStartPoint.getY() / Conveyor.RASTER_SIZE);
		myStopPoint = new Point(newStopPoint.getX() / Conveyor.RASTER_SIZE, newStopPoint.getY() / Conveyor.RASTER_SIZE);
		
		//System.out.println("Start: " + myStartPoint.toString());
		//System.out.println("Stop : " + myStopPoint.toString());
		
		if(myStartPoint.getX() < 0 || myStartPoint.getY() < 0) {
			returnValue = PathMessageType.PathError;
			return null;
		}
		
		if(myStopPoint.getX() < 0 || myStopPoint.getY() < 0) {
			returnValue = PathMessageType.PathError;	
			return null;
		}	
		
		if (lstGridItem == null) {
			returnValue = PathMessageType.NotInitialized;
			return null;
		}
		
		lstGridItem.get((getIndex(myStartPoint, myColumnCount))).setItemType(GridItemType.StartItem);
        lstGridItem.get((getIndex(myStopPoint, myColumnCount))).setItemType(GridItemType.StopItem);
		
        if(myStartPoint.getX() == myStopPoint.getX() && myStartPoint.getY() == myStopPoint.getY())
        	returnValue = PathMessageType.StartEndPointsEqual;
        
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

            minValue = saveBestPoint(curPathPoint, Direction.Left, minValue, lstPossiblePoints);
            minValue = saveBestPoint(curPathPoint, Direction.Top, minValue, lstPossiblePoints);
            minValue = saveBestPoint(curPathPoint, Direction.Right, minValue, lstPossiblePoints);
            minValue = saveBestPoint(curPathPoint, Direction.Bottom, minValue, lstPossiblePoints);
    	            
            if(bDriveDiagonal){
            	minValue = saveBestPoint(curPathPoint, Direction.TopLeft, minValue, lstPossiblePoints);
            	minValue = saveBestPoint(curPathPoint, Direction.TopRight, minValue, lstPossiblePoints);
            	minValue = saveBestPoint(curPathPoint, Direction.BottomLeft, minValue, lstPossiblePoints);
            	minValue = saveBestPoint(curPathPoint, Direction.BottomRight, minValue, lstPossiblePoints);
            }
            
            //System.out.println("PossibleSize: " + lstPossiblePoints.size());
            
            if (lstPossiblePoints.size() > 0){
            	int randomIndex = (int)(Math.random() * 100) % lstPossiblePoints.size();
            	PathPoint lessRotatingPathPoint = null;
            	
            	if(bAvoidRotations){
            		for(PathPoint myPathPoint : lstPossiblePoints){
            			if(curPathPoint.Direction == myPathPoint.Direction){
            				lessRotatingPathPoint = myPathPoint;
            				break;
            			}
            		}
            	}
            	
            	if(lessRotatingPathPoint == null)
            		curPathPoint = lstPossiblePoints.get(randomIndex);
            	else
            		curPathPoint = lessRotatingPathPoint;
            	
            	lstPossiblePoints.clear();
            	
            	lstSinglePathpoints.add(curPathPoint);
            	
                if(curPathPoint.getPoint().getX() == myStartPoint.getX() && curPathPoint.getPoint().getY() == myStartPoint.getY()){
                	break;
                }
                
        		lstGridItem.get(getIndex(curPathPoint.getPoint(), myColumnCount)).setItemType(GridItemType.PathItem);
            }
            else {
            	lstSinglePathpoints.clear();
            	
            	resetValues();
            	
            	/*if(bDriveDiagonal){
            		bRunning = false;
            		bDriveDiagonal = false;
            		lstPathPoints.clear();
            		returnValue = findPath(myStartPoint, myStopPoint);
            		bDriveDiagonal = true;
            		
            	} else{
            		returnValue = PathMessageType.PathBlocked;
            	}
            	break;*/            
            }
        }
        
        Collections.reverse(lstSinglePathpoints);
        
    	lstPathPoints.add(lstSinglePathpoints);
    	
    	bRunning = false;
        
		return lstPathPoints;
	}
}
