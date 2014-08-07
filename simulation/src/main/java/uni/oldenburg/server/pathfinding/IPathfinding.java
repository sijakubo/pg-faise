package uni.oldenburg.server.pathfinding;

import java.util.List;

import uni.oldenburg.server.pathfinding.Pathfinding.PathMessageType;
import uni.oldenburg.shared.model.Point;
/**
 * Interface für das Pathfinding. Alle Klassen, die einen Pathfinding-Algorithmus implementieren, müssen die Methoden des Interfaces 
 * implementieren, damit der Pathfinding-Algorithmus ausgetauscht werden kann, ohne das Änderungen an den Agenten bzw.Behaviours vorgenommen werden müssen.  
 * @author Matthias
 */	
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
	List<List<PathPoint> > findPath(Point newStartPoint, Point newStopPoint);
	PathMessageType getError();
}
