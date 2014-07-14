package uni.oldenburg.server.pathfinding;

public class GridItem {
	int gridSize;
	GridItemType type;
	int myGridValue;
	int myStepValue;
	
	public enum GridItemType {
		DefaultItem,
		StartItem,
		StopItem,
		WallItem,
		PathItem				
	}

	public GridItem(int newGridItemSize) {
		this.gridSize = newGridItemSize;
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
