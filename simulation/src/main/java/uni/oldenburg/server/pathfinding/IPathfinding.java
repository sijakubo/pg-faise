package uni.oldenburg.server.pathfinding;

import java.util.List;

import uni.oldenburg.server.pathfinding.Pathfinding.PathMessageType;

import com.google.gwt.touch.client.Point;

public interface IPathfinding {
	boolean isRunning();
	int getColumnCount();
	int getRowCount();
	Point getStartPoint();
	Point getStopPoint();
	
	boolean getDriveDiagonal();
	void setDriveDiagonal(boolean value);
	
	boolean getAvoidRotations();
	void setAvoidRotations(boolean value);
	
	void init (int columnCount, int rowCount, List<GridItem> lstGridItem);
	PathMessageType findPath(Point newStartPoint, Point newStopPoint, List<List<PathPoint> > lstPathPoints);
}
