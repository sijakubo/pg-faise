package uni.oldenburg.client.presenter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import uni.oldenburg.client.service.SimulationServiceAsync;
import uni.oldenburg.client.view.DialogBoxScenarioSelection;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.ConveyorVehicle;
import uni.oldenburg.shared.model.Job;
import uni.oldenburg.shared.model.Szenario;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.canvas.dom.client.Context2d;
import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.cellview.client.CellTable;
import com.google.gwt.user.cellview.client.TextColumn;
import com.google.gwt.user.client.Command;
import com.google.gwt.user.client.Random;
import com.google.gwt.user.client.Window;
import com.google.gwt.user.client.rpc.AsyncCallback;
import com.google.gwt.user.client.ui.MenuBar;
import com.google.gwt.user.client.ui.Widget;
import com.google.gwt.view.client.AsyncDataProvider;
import com.google.gwt.view.client.HasData;

public class MainFramePresenter extends Presenter {
	private final IDisplay display;

	private static final List<Job> JOBS = Arrays.asList(new Job(1, "XY"),
			new Job(2, "zt"), new Job(3, "Patata"), new Job(4, "UO"));

	public interface IDisplay {
		CellTable<Job> getJobTable();

		HasClickHandlers getStrategiesButton();

		HasClickHandlers getVirtualHybridButton();

		HasClickHandlers getConveyorRampButton();

		HasClickHandlers getConveyorVehicleButton();

		MenuBar getMenuBar();

		MenuBar getFileMenuBar();

		MenuBar getEditMenuBar();

		MenuBar getViewMenuBar();

		Canvas getCanvas();
	}

	public MainFramePresenter(SimulationServiceAsync rpcService,
			HandlerManager eventBus, IDisplay view) {
		super(rpcService, eventBus);
		this.display = view;
	}

	public Widget getDisplay() {
		return (Widget) display;
	}

	private void addStrategiesButtonListener() {
		display.getStrategiesButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				// TODO
			}
		});
	}

	private void addVirtualHybridButtonListener() {
		display.getVirtualHybridButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				// TODO
			}
		});
	}

	private void addConveyorRampButtonListener() {
		display.getConveyorRampButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				// TODO
			}
		});
	}

	private void addConveyorVehicleButtonListener() {
		display.getConveyorVehicleButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				// TODO
			}
		});
	}

	private void setupJobTable() {
		TextColumn<Job> jobNumberColumn = new TextColumn<Job>() {
			@Override
			public String getValue(Job object) {
				return "" + object.getJobNumber();
			}
		};

		TextColumn<Job> jobColumn = new TextColumn<Job>() {
			@Override
			public String getValue(Job object) {
				return object.getJob();
			}
		};

		display.getJobTable().addColumn(jobNumberColumn, "Job Number");
		display.getJobTable().addColumn(jobColumn, "Job");

		AsyncDataProvider<Job> provider = new AsyncDataProvider<Job>() {
			@Override
			protected void onRangeChanged(HasData<Job> display) {
				int start = display.getVisibleRange().getStart();
				int end = start + display.getVisibleRange().getLength();
				end = end >= JOBS.size() ? JOBS.size() : end;
				List<Job> sub = JOBS.subList(start, end);
				updateRowData(start, sub);
			}
		};
		provider.addDataDisplay(display.getJobTable());
		provider.updateRowCount(JOBS.size(), true);
	}

	private void drawConveyor(Conveyor myConveyer) {
		Context2d context = display.getCanvas().getContext2d();
		context.drawImage(myConveyer.getCanvasElement(), myConveyer.getX(),
				myConveyer.getY());
	}

	@SuppressWarnings("unused")
	private void generateConveyor() {
		for (int i = 0; i < 5; ++i) {
			drawConveyor(new ConveyorRamp(Random.nextInt(780),
					Random.nextInt(460)));
		}

		for (int i = 0; i < 5; ++i) {
			drawConveyor(new ConveyorVehicle(Random.nextInt(780),
					Random.nextInt(460)));
		}
	}

	public void loadSzenario(String name) {
		((SimulationServiceAsync) rpcService).loadSzenario(name,
				new AsyncCallback<Szenario>() {
					public void onFailure(Throwable arg0) {
						Window.alert(arg0.getLocalizedMessage());
					}

					public void onSuccess(Szenario szenario) {
						List<Conveyor> lstConveyor = szenario.getConveyorList();

						if (lstConveyor.size() == 0) {
							Window.alert("Keine Stetigförderer im Szenario gefunden!");
							return;
						}

						for (Conveyor myConveyor : lstConveyor) {
							drawConveyor(myConveyor);
						}
					}
				});
	}
	
	
	//Gets the Scenario Titles From Server and displays it in a Dialog
	public void getScenarioTitlesFromServerAndShow(){
		
		
		((SimulationServiceAsync) rpcService).getScenarioTitles(
				new AsyncCallback<ArrayList<String>>() {
					
					
					
					public void onFailure(Throwable arg0) {
						Window.alert(arg0.getLocalizedMessage());
					}

					

					public void onSuccess(ArrayList<String> result) {
						
						DialogBoxScenarioSelection dialog=new DialogBoxScenarioSelection(result,MainFramePresenter.this); 
						dialog.show();
						
						
					}
				});
		
		
		
	}
	
	
	
	
	
	
	
	

	public void bind() {
		this.initializeMenuBars();
		this.addStrategiesButtonListener();
		this.addVirtualHybridButtonListener();
		this.addConveyorRampButtonListener();
		this.addConveyorVehicleButtonListener();
		this.setupJobTable();
		// this.generateConveyor();
		loadSzenario("TestSzenario");
	}

	// Method initializes the Menubar. This is done here, because it is necessary to embed Client-Server Communication
	// into the Execute-methods of the Menubaritems. It would be ugly to insert the RPC-Service into a View 
	private void initializeMenuBars() {
		// --- menu bar ---

		// file menu

		
		this.display.getFileMenuBar().addItem("Load Scenario", new Command() {
			public void execute() {
                   getScenarioTitlesFromServerAndShow();
			}
		});
		this.display.getFileMenuBar().addItem("Recently Used", new Command() {
			public void execute() {

			}
		});
		this.display.getFileMenuBar().addSeparator();
		this.display.getFileMenuBar().addItem("Adjust Scenario", new Command() {
			public void execute() {

			}
		});
		this.display.getFileMenuBar().addItem("Save", new Command() {
			public void execute() {

			}
		});
		this.display.getFileMenuBar().addItem("Save As", new Command() {
			public void execute() {

			}
		});
		this.display.getFileMenuBar().addSeparator();
		this.display.getFileMenuBar().addItem("Print", new Command() {
			public void execute() {

			}
		});
		this.display.getFileMenuBar().addItem("Exit", new Command() {
			public void execute() {

			}
		});

		// edit menu

		
		this.display.getEditMenuBar().addItem("Undo", new Command() {
			public void execute() {

			}
		});
		this.display.getEditMenuBar().addItem("Redo", new Command() {
			public void execute() {

			}
		});
		this.display.getEditMenuBar().addSeparator();
		this.display.getEditMenuBar().addItem("Define Job Set", new Command() {
			public void execute() {

			}
		});
		this.display.getEditMenuBar().addItem("Edit Jobs", new Command() {
			public void execute() {

			}
		});
		this.display.getEditMenuBar().addItem("Change Database", new Command() {
			public void execute() {

			}
		});
		this.display.getEditMenuBar().addItem("Simulation Settings", new Command() {
			public void execute() {

			}
		});
		this.display.getEditMenuBar().addItem("Start/Stop Simulation", new Command() {
			public void execute() {

			}
		});
		this.display.getEditMenuBar().addSeparator();
		this.display.getEditMenuBar().addItem("Save As Image", new Command() {
			public void execute() {

			}
		});
		this.display.getEditMenuBar().addItem("PlugIn", new Command() {
			public void execute() {

			}
			// hey
		});

		// view menu

		
		this.display.getViewMenuBar().addItem("Brightness", new Command() {
			public void execute() {

			}
		});
		this.display.getViewMenuBar().addSeparator();
		this.display.getViewMenuBar().addItem("Zoom In", new Command() {
			public void execute() {

			}
		});
		this.display.getViewMenuBar().addItem("Zoom Out", new Command() {
			public void execute() {

			}
		});
		this.display.getViewMenuBar().addSeparator();
		this.display.getViewMenuBar().addItem("Fullscreen", new Command() {
			public void execute() {

			}
		});

		// menu bar

		
		this.display.getMenuBar().addItem("File", this.display.getFileMenuBar());
		this.display.getMenuBar().addSeparator();
		this.display.getMenuBar().addItem("Edit", this.display.getEditMenuBar());
		this.display.getMenuBar().addSeparator();
		this.display.getMenuBar().addItem("View", this.display.getViewMenuBar());
		this.display.getMenuBar().addSeparator();
		this.display.getMenuBar().addItem("Help", new Command() {
			public void execute() {

			}
		});
	}

}
