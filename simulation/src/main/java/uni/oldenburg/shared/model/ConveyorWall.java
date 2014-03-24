package uni.oldenburg.shared.model;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.canvas.dom.client.Context2d;
import com.google.gwt.canvas.dom.client.CssColor;

@SuppressWarnings("serial")
public class ConveyorWall extends Conveyor {
	public static final String TYPE = "Wand";
	
	public ConveyorWall() {
		super(0, 0, Conveyor.getRastersize(), Conveyor.getRastersize());
	}
	
	public ConveyorWall(int x, int y) {
		super(x, y, Conveyor.getRastersize(), Conveyor.getRastersize());
	}

	@Override
	protected Canvas createCanvas(Canvas canvas) {
		Context2d context = canvas.getContext2d();
		
		context.setFillStyle(CssColor.make("black"));
		context.fillRect(0, 0, getWidth(), getHeight());
		context.fill();
		
		return canvas;
	}

	@Override
	public String getType() {
		return ConveyorWall.TYPE;
	}

}
