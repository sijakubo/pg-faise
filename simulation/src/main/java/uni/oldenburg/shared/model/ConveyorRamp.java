package uni.oldenburg.shared.model;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.canvas.dom.client.Context2d;
import com.google.gwt.canvas.dom.client.CssColor;

/**
 * class for handing ramp blocks in the canvas
 * 
 * @author Matthias
 */

@SuppressWarnings("serial")
public class ConveyorRamp extends Conveyor {
	public static final String TYPE = "Rampe";
	private boolean bVertical = true;
	private int numBlocks = 3;

	public ConveyorRamp() {
		setSize(Conveyor.getRastersize(), Conveyor.getRastersize() * numBlocks);
	}
	
	public ConveyorRamp(int x, int y) {
		setPosition(x, y);
		setSize(Conveyor.getRastersize(), Conveyor.getRastersize() * numBlocks);
	}

	@Override
	protected Canvas createCanvas(Canvas canvas) {
		Context2d context = canvas.getContext2d();
		
		context.setFillStyle(CssColor.make("red"));
		context.fillRect(0, 0, getWidth(), getHeight());
		context.fill();
		
		return canvas;
	}

	@Override
	public String getType() {
		return ConveyorRamp.TYPE;
	}

	public boolean isVertical() {
		return bVertical;
	}
	
	/**
	 * change size based on vertical status
	 * 
	 * @author Matthias
	 */	

	public void setVertical(boolean value) {
		if (this.bVertical == value)
			return;
		
		this.bVertical = value;
		
		if (value)
			this.setSize(Conveyor.getRastersize(), Conveyor.getRastersize() * numBlocks);
		else
			this.setSize(Conveyor.getRastersize() * numBlocks, Conveyor.getRastersize());
	}
}
