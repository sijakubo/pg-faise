package uni.oldenburg.server.pathfinding;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import uni.oldenburg.server.pathfinding.GridItem.GridItemType;
import uni.oldenburg.shared.model.Point;
/**
 * Implementiert das eigentliche Pathfinding.
 * 
 * @author Matthias
 */	
public class PathfindingSingle extends Pathfinding {
	List<List<PathPoint>> lstPathPoints = new ArrayList<List<PathPoint>>();
	
	PathMessageType returnValue = PathMessageType.Running;
	
	public PathfindingSingle() {};
	
	public PathfindingSingle(int columnCount, int rowCount, List<GridItem> lstGridItem) {
		init(columnCount, rowCount, lstGridItem);
	};
	
	
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
		
		myStartPoint = newStartPoint;
		myStopPoint = newStopPoint;
		
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
		
        if(myStartPoint.getX() == myStopPoint.getX() && myStartPoint.getY() == myStopPoint.getY()) {
        	returnValue = PathMessageType.StartEndPointsEqual;
        	return null;
        }
        	
        bRunning = true;
        
        ComputeBlocksValues();
    
        drawGrid(myColumnCount, myRowCount, lstGridItem);
        
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
            
            myMinValue = minValue;

            lstPossiblePoints = saveBestPoint(curPathPoint, Direction.Left, lstPossiblePoints);
            lstPossiblePoints = saveBestPoint(curPathPoint, Direction.Top, lstPossiblePoints);
            lstPossiblePoints = saveBestPoint(curPathPoint, Direction.Right, lstPossiblePoints);
            lstPossiblePoints = saveBestPoint(curPathPoint, Direction.Bottom, lstPossiblePoints);
    	            
            if(bDriveDiagonal){
            	lstPossiblePoints = saveBestPoint(curPathPoint, Direction.TopLeft, lstPossiblePoints);
            	lstPossiblePoints = saveBestPoint(curPathPoint, Direction.TopRight, lstPossiblePoints);
            	lstPossiblePoints = saveBestPoint(curPathPoint, Direction.BottomLeft, lstPossiblePoints);
            	lstPossiblePoints = saveBestPoint(curPathPoint, Direction.BottomRight, lstPossiblePoints);
            }
            
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
        		
        		//drawGrid(myColumnCount, myRowCount, lstGridItem);
            }
            else {
            	lstSinglePathpoints.clear();
            	
            	resetValues();
            	
            	if(bDriveDiagonal){
            		bRunning = false;
            		bDriveDiagonal = false;
            		lstPathPoints.clear();
            		lstPathPoints = findPath(myStartPoint, myStopPoint);
            		bDriveDiagonal = true;
            		
            	} else{
            		returnValue = PathMessageType.PathBlocked;
            	}
            	break;            
            }
        }
        
        Collections.reverse(lstSinglePathpoints);
        
    	this.lstPathPoints.add(lstSinglePathpoints);
    	
    	bRunning = false;
        
		return this.lstPathPoints;
	}

	public PathMessageType getError() {
		return returnValue;
	}
}
