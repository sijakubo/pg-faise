package uni.oldenburg.server.pathfinding;

import java.util.List;

import uni.oldenburg.server.pathfinding.Pathfinding.PathMessageType;
import uni.oldenburg.shared.model.Point;

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
	List<List<PathPoint>> findPath(Point newStartPoint, Point newStopPoint);
	PathMessageType getStatus();
}
