package uni.oldenburg.client.view;

import uni.oldenburg.client.presenter.MainFramePresenter;
import uni.oldenburg.shared.model.Job;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.user.cellview.client.CellTable;
import com.google.gwt.user.cellview.client.SimplePager;
import com.google.gwt.user.client.ui.Button;
import com.google.gwt.user.client.ui.Composite;
import com.google.gwt.user.client.ui.Label;
import com.google.gwt.user.client.ui.MenuBar;
import com.google.gwt.user.client.ui.HorizontalPanel;
import com.google.gwt.user.client.ui.ToggleButton;
import com.google.gwt.user.client.ui.VerticalPanel;

public class MainFrameView extends Composite implements MainFramePresenter.IDisplay {
	private MenuBar			menuBar;	
	private MenuBar			simMenuBar;
	private MenuBar			editMenuBar;
	
	private CellTable<Job>	ctJobTable = new CellTable<Job>();
	private Button         	btnStrategies;
	private ToggleButton	tbVirtualHybridSwitch;
	// conveyor menu
	private Label			lblConveyor;
	private Button			btnConveyorRamp;
	private Button			btnConveyorVehicle;
	// -------------
	
	private Canvas			canvas;
	
	private final int canvasWidth  = 800;
	private final int canvasHeight = 480;
	
	public MainFrameView() {

		VerticalPanel   vpMainFrame = new VerticalPanel();
		HorizontalPanel hpSubFrame = new HorizontalPanel();
		VerticalPanel   vpLeftFrame = new VerticalPanel();
		vpLeftFrame.addStyleName("left_panel");
			
		//--- menu bar ---
		
		//simulation menu
		
		simMenuBar = new MenuBar(true);
		simMenuBar.setAnimationEnabled(true);
		
		//edit menu
		
		editMenuBar = new MenuBar(true);
		editMenuBar.setAnimationEnabled(true);
		
		//menu bar
		
		menuBar = new MenuBar();
	
		//--- left panel ---

		//table
		
		ctJobTable.setPageSize(5);
	    vpLeftFrame.add(ctJobTable);

	    SimplePager pager = new SimplePager();
	    pager.setDisplay(ctJobTable);

	    VerticalPanel vp = new VerticalPanel();
	    vp.add(ctJobTable);
	    vp.add(pager);
	    vpLeftFrame.add(vp);
	    
	    //togglebutton for switching between hybrid and virtual mode
	    
		tbVirtualHybridSwitch = new ToggleButton("Virtual / Hybrid");
		tbVirtualHybridSwitch.setText("Virtual / Hybrid");
		vpLeftFrame.add(tbVirtualHybridSwitch);
		
		//button for changing pathfinding strategy
		
		btnStrategies = new Button("Strategies");
		btnStrategies.setText("Strategies");
		vpLeftFrame.add(btnStrategies);
		
		// conveyor list
		VerticalPanel vpConveyor = new VerticalPanel();
		vpConveyor.addStyleName("conveyor_panel");
		
		lblConveyor = new Label("Stetigfoerderer");
		lblConveyor.addStyleName("conveyor_label");
		btnConveyorRamp = new Button("Rampe");
		btnConveyorRamp.addStyleName("conveyor_ramp");
		btnConveyorVehicle = new Button("Fahrzeug");
		btnConveyorVehicle.addStyleName("conveyor_vehicle");
		
		vpConveyor.add(lblConveyor);
		vpConveyor.add(btnConveyorRamp);
		vpConveyor.add(btnConveyorVehicle);
		
		vpLeftFrame.add(vpConveyor);
		// -------------
		
		hpSubFrame.add(vpLeftFrame);
		
		// create canvas object
		canvas = Canvas.createIfSupported();
		canvas.setWidth(canvasWidth + "px");
		canvas.setHeight(canvasHeight + "px");
		canvas.setCoordinateSpaceWidth(canvasWidth);
		canvas.setCoordinateSpaceHeight(canvasHeight);
		hpSubFrame.add(canvas);
		
		vpMainFrame.add(menuBar);
		vpMainFrame.add(hpSubFrame);
		initWidget(vpMainFrame);
	}

    public HasClickHandlers getStrategiesButton() {
        return btnStrategies;
    }

    public HasClickHandlers getVirtualHybridButton() {
        return tbVirtualHybridSwitch;
    }
    
    public CellTable<Job> getJobTable() {
        return ctJobTable;
    }

	public Canvas getCanvas() {
		return canvas;
	}

	public HasClickHandlers getConveyorRampButton() {
		return btnConveyorRamp;
	}

	public HasClickHandlers getConveyorVehicleButton() {
		return btnConveyorVehicle;
	}

	public MenuBar getMenuBar() {
		return this.menuBar;
	}

	public MenuBar getSimulationMenuBar() {
		return this.simMenuBar;
	}

	public MenuBar getEditMenuBar() {
		return this.editMenuBar;
	}
}