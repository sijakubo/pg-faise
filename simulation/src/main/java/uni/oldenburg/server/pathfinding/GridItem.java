package uni.oldenburg.server.pathfinding;
/**
 * Klasse repräsentiert eine Kachel auf der Karte.
 * @author Matthias
 */	
public class GridItem {
	int gridSize;//Größe der Kachel
	GridItemType type;//Typ der Kachel (Mauer, leere Kachel etc. siehe Enum)
	int myGridValue;//Wert der hochgezählt wird, wenn bei der Vorwärtsberechnung ein Pfad gesucht wird.
	int myStepValue;//Entfernung zur nächsten Kachel
	
	public enum GridItemType {
		DefaultItem,
		StartItem,
		StopItem,
		WallItem,
		PathItem				
	}

	public GridItem(int newGridItemSize) {
		this.gridSize = newGridItemSize;
		setItemType(GridItemType.DefaultItem, true);
	}
	
	public int getGridValue() {
		return myGridValue;
	}

	public void setGridValue(int myGridValue) {
		this.myGridValue = myGridValue;
	}

	public int getStepValue() {
		return myStepValue;
	}

	public void setStepValue(int myStepValue) {
		this.myStepValue = myStepValue;
	}
	
	public GridItemType getItemType() {
		return this.type;
	}
	
	public void setItemType(GridItemType newType) {
		setItemType(newType, false);
	}
	//Methode 
	public void setItemType(GridItemType newType, boolean reseting) {
		if (!reseting) {
			if (newType == GridItemType.WallItem && getItemType() == GridItemType.WallItem) {
				newType = GridItemType.DefaultItem;
			}
		}
		
		type = newType;
		
		switch(type) {
			case DefaultItem:
				myGridValue = -1;
				break;
			case StartItem:
				myGridValue = 0;
				break;
			case StopItem:
				myGridValue = -2;
				break;
			case WallItem:
				myGridValue = -3;
				break;
			default:
				break;
		}
	}
}
