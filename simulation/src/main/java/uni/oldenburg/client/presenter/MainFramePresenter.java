package uni.oldenburg.client.presenter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import uni.oldenburg.client.service.ServiceAsync;
import uni.oldenburg.client.service.SimulationServiceAsync;
import uni.oldenburg.client.view.DialogBoxOverwrite;
import uni.oldenburg.client.view.DialogBoxSaveAs;
import uni.oldenburg.client.view.DialogBoxScenarioSelection;
import uni.oldenburg.shared.model.Conveyor;
import uni.oldenburg.shared.model.ConveyorRamp;
import uni.oldenburg.shared.model.ConveyorVehicle;
import uni.oldenburg.shared.model.ConveyorWall;
import uni.oldenburg.shared.model.Job;
import uni.oldenburg.shared.model.Szenario;

import com.google.gwt.canvas.client.Canvas;
import com.google.gwt.canvas.dom.client.Context2d;
import com.google.gwt.canvas.dom.client.CssColor;
import com.google.gwt.dom.client.NativeEvent;
import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.event.dom.client.ContextMenuEvent;
import com.google.gwt.event.dom.client.ContextMenuHandler;
import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.event.dom.client.KeyCodes;
import com.google.gwt.event.dom.client.KeyUpEvent;
import com.google.gwt.event.dom.client.KeyUpHandler;
import com.google.gwt.event.dom.client.MouseMoveEvent;
import com.google.gwt.event.dom.client.MouseMoveHandler;
import com.google.gwt.event.dom.client.MouseUpEvent;
import com.google.gwt.event.dom.client.MouseUpHandler;
import com.google.gwt.event.shared.HandlerManager;
import com.google.gwt.user.cellview.client.CellTable;
import com.google.gwt.user.cellview.client.TextColumn;
import com.google.gwt.user.client.Command;
import com.google.gwt.user.client.Random;
import com.google.gwt.user.client.Window;
import com.google.gwt.user.client.rpc.AsyncCallback;
import com.google.gwt.user.client.ui.DialogBox;
import com.google.gwt.user.client.ui.MenuBar;
import com.google.gwt.user.client.ui.Widget;
import com.google.gwt.view.client.AsyncDataProvider;
import com.google.gwt.view.client.HasData;

public class MainFramePresenter extends Presenter {
	private final IDisplay display;
	private Szenario currentSzenario;
	Conveyor dropableConveyor;

	private static final List<Job> JOBS = Arrays.asList(new Job("XY",
			Job.OUTGOING, 42, 12, 128),
			new Job("zt", Job.OUTGOING, 73, 45, 256), new Job("Patata",
					Job.INCOMING, 128, 64, 512), new Job("UO", Job.OUTGOING,
					1337, 128, 1024));

	public interface IDisplay {
		CellTable<Job> getJobTable();

		HasClickHandlers getStrategiesButton();

		HasClickHandlers getVirtualHybridButton();

		HasClickHandlers getConveyorRampButton();

		HasClickHandlers getConveyorVehicleButton();

		HasClickHandlers getConveyorWallButton();

		MenuBar getMenuBar();

		MenuBar getSimulationMenuBar();

		MenuBar getEditMenuBar();

		Canvas getCanvas();
	}

	public MainFramePresenter(SimulationServiceAsync rpcService,
			HandlerManager eventBus, IDisplay view) {
		super(rpcService, eventBus);
		this.display = view;
		this.currentSzenario = new Szenario();
		this.dropableConveyor = null;
	}

	public Widget getDisplay() {
		return (Widget) display;

	}

	private void addCanvasListener() {
		// standard-kontextmenu im canvas object nicht unterbinden
		display.getCanvas().addDomHandler(new ContextMenuHandler() {
			public void onContextMenu(ContextMenuEvent event) {
				event.preventDefault();
				event.stopPropagation();
			}
		}, ContextMenuEvent.getType());

		display.getCanvas().addMouseMoveHandler(new MouseMoveHandler() {
			public void onMouseMove(MouseMoveEvent event) {
				Conveyor myConveyor = MainFramePresenter.this.dropableConveyor;

				if (myConveyor == null)
					return;

				myConveyor.setPosition(event.getX(), event.getY());

				loadSzenario(MainFramePresenter.this.currentSzenario);
				drawConveyor(myConveyor);

				display.getCanvas().setFocus(true);
			}
		});

		display.getCanvas().addMouseUpHandler(new MouseUpHandler() {
			public void onMouseUp(MouseUpEvent event) {
				Conveyor myConveyor = MainFramePresenter.this.dropableConveyor;

				// conveyor drag & drop
				if (event.getNativeButton() == NativeEvent.BUTTON_LEFT) {
					// conveyor hinzufügen
					if (myConveyor != null) {
						// wenn platz noch frei ist
						if (isSpotAvailable(event.getX(), event.getY())) {
							MainFramePresenter.this.currentSzenario
									.addConveyor(myConveyor);
							MainFramePresenter.this.dropableConveyor = null;
							loadSzenario(MainFramePresenter.this.currentSzenario);
						}
					} else {
						// conveyor verschieben
						grabConveyor(event.getX(), event.getY());
					}
				} else if (event.getNativeButton() == NativeEvent.BUTTON_RIGHT) {
					// rampe rotieren
					if (myConveyor == null)
						return;

					if (myConveyor.getType().compareTo("Rampe") == 0) {
						((ConveyorRamp) myConveyor)
								.setVertical(!((ConveyorRamp) myConveyor)
										.isVertical());

						loadSzenario(MainFramePresenter.this.currentSzenario);
						drawConveyor(myConveyor);

						display.getCanvas().setFocus(true);
					}
				}
			}
		});

		display.getCanvas().addKeyUpHandler(new KeyUpHandler() {
			public void onKeyUp(KeyUpEvent event) {
				if (event.getNativeKeyCode() == KeyCodes.KEY_ESCAPE) {
					MainFramePresenter.this.dropableConveyor = null;
					loadSzenario(MainFramePresenter.this.currentSzenario);
				}
			}
		});
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
				MainFramePresenter.this.dropableConveyor = new ConveyorRamp();
			}
		});
	}

	private void addConveyorVehicleButtonListener() {
		display.getConveyorVehicleButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				MainFramePresenter.this.dropableConveyor = new ConveyorVehicle();
			}
		});
	}

	private void addConveyorWallButtonListener() {
		display.getConveyorWallButton().addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				MainFramePresenter.this.dropableConveyor = new ConveyorWall();
			}
		});
	}

	private void setupJobTable() {
		TextColumn<Job> jobIdColumn = new TextColumn<Job>() {
			@Override
			public String getValue(Job object) {
				return "" + object.getId();
			}
		};

		TextColumn<Job> jobColumn = new TextColumn<Job>() {
			@Override
			public String getValue(Job object) {
				return object.getJob();
			}
		};

		display.getJobTable().addColumn(jobIdColumn, "Auftrags-Id");
		display.getJobTable().addColumn(jobColumn, "Auftrag");

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

	private void drawConveyor(Conveyor myConveyor) {
		Context2d context = display.getCanvas().getContext2d();
		context.drawImage(myConveyor.getCanvasElement(), myConveyor.getX(),
				myConveyor.getY());
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

	public boolean isSpotAvailable(int x, int y) {
		return (findConveyor(x, y) == null);
	}

	public Conveyor findConveyor(int x, int y) {
		Conveyor myConveyor = null;
		List<Conveyor> lstConveyor = this.currentSzenario.getConveyorList();

		for (Conveyor cvEntry : lstConveyor) {
			if (x < cvEntry.getX())
				continue;
			if (y < cvEntry.getY())
				continue;
			if (x > (cvEntry.getX() + cvEntry.getWidth()))
				continue;
			if (y > (cvEntry.getY() + cvEntry.getHeight()))
				continue;

			myConveyor = cvEntry;
			break;
		}

		return myConveyor;
	}

	public void grabConveyor(int x, int y) {
		Conveyor myConveyor = findConveyor(x, y);

		if (myConveyor != null)
			this.currentSzenario.removeConveyor(myConveyor);

		this.dropableConveyor = myConveyor;
	}

	public void clearCanvas() {
		Context2d context = display.getCanvas().getContext2d();
		context.setFillStyle(CssColor.make(255, 255, 255));
		context.fillRect(0, 0, display.getCanvas().getCoordinateSpaceWidth(),
				display.getCanvas().getCoordinateSpaceHeight());
		context.fill();

	}

	public void loadSzenario(Szenario szenario) {
		clearCanvas();

		if (szenario == null)
			return;

		List<Conveyor> lstConveyor = szenario.getConveyorList();

		for (Conveyor myConveyor : lstConveyor) {
			drawConveyor(myConveyor);
		}
	}

	public void loadSzenario(String name) {
		((SimulationServiceAsync) rpcService).loadSzenario(name,
				new AsyncCallback<Szenario>() {
					public void onFailure(Throwable arg0) {
						Window.alert(arg0.getLocalizedMessage());
					}

					public void onSuccess(Szenario szenario) {
						MainFramePresenter.this.currentSzenario = szenario;
						loadSzenario(szenario);
					}
				});
	}

	/**
	 * Gets the Scenario Titles From Server and displays it in a Dialog
	 * 
	 * @author Raschid
	 */
	public void getScenarioTitlesFromServerAndShow() {
		((SimulationServiceAsync) rpcService)
				.getScenarioTitles(new AsyncCallback<ArrayList<String>>() {
					public void onFailure(Throwable arg0) {
						Window.alert(arg0.getLocalizedMessage());
					}

					public void onSuccess(ArrayList<String> result) {
						DialogBoxScenarioSelection dialog = new DialogBoxScenarioSelection(
								result, MainFramePresenter.this);
						dialog.show();
					}
				});
	}

	/**
	 * Method checks if Szenario exists and then decides wether to save or to
	 * open the Save As Dialog
	 * 
	 * @author Raschid
	 */
	public void trySaveSzenario(Szenario szenario) {

		// Check if Szenario already exists
		((SimulationServiceAsync) rpcService).checkIfTitleExists(
				szenario.getTitle(), new AsyncCallback<Boolean>() {

					public void onFailure(Throwable caught) {
						Window.alert("Communication Problem");

					}

					public void onSuccess(Boolean result) {
						// If Szenario exists open a popup, where the user can
						// be asked if he wants to overwritte or not
						if (result) {
							// Show the popup so the user can select if he wants
							// to overwrite or not
							DialogBoxOverwrite dialog = new DialogBoxOverwrite(
									MainFramePresenter.this);
							dialog.show();
						} else {
							// Open Save as Dialog
							DialogBoxSaveAs dialog = new DialogBoxSaveAs(
									MainFramePresenter.this);
							dialog.show();
						}

					}

				});

	}

	/**
	 * Method sends the Szenario to the server in order to write it into the
	 * database. The operation String decides wether an Update or an Insert
	 * should be made
	 * 
	 * @author Raschid
	 */
	public void sendSzenarioToServer(Szenario szenario, String operation) {

		// Send Szenario to Server
		((SimulationServiceAsync) rpcService).saveSzenario(szenario, operation,
				new AsyncCallback<Void>() {

					public void onFailure(Throwable caught) {
						Window.alert("Communication Problem, Szenario was not saved");

					}

					public void onSuccess(Void result) {
						Window.alert("Szenario was successfully saved");

					}

				});
	}

	public void bind() {
		this.initializeMenuBars();
		this.addStrategiesButtonListener();
		this.addVirtualHybridButtonListener();
		this.addConveyorRampButtonListener();
		this.addConveyorVehicleButtonListener();
		this.addConveyorWallButtonListener();
		this.addCanvasListener();
		this.setupJobTable();

	}

	/**
	 * Method initializes the Menubar. This is done here, because it is
	 * necessary to embed Client-Server Communication into the Execute-methods
	 * of the Menubaritems. It would be ugly to insert the RPC-Service into a
	 * View
	 * 
	 * @author Raschid
	 */
	private void initializeMenuBars() {
		// --- menu bar ---

		// file menu

		this.display.getSimulationMenuBar().addItem("Load Scenario",
				new Command() {
					public void execute() {
						getScenarioTitlesFromServerAndShow();
					}
				});

		this.display.getSimulationMenuBar().addItem("Save", new Command() {
			public void execute() {
				trySaveSzenario(MainFramePresenter.this.getActualSzenario());
			}
		});
		this.display.getSimulationMenuBar().addItem("Save As", new Command() {
			public void execute() {
				// Open Save as Dialog
				DialogBoxSaveAs dialog = new DialogBoxSaveAs(
						MainFramePresenter.this);
				dialog.show();
			}
		});

		this.display.getSimulationMenuBar().addSeparator();

		this.display.getSimulationMenuBar().addItem("Simulation Settings",
				new Command() {
					public void execute() {

					}
				});

		this.display.getSimulationMenuBar().addSeparator();

		this.display.getSimulationMenuBar().addItem("Start/Stop Simulation",
				new Command() {
					public void execute() {

					}
				});

		// edit menu
		this.display.getEditMenuBar().addItem("Define Job Set", new Command() {
			public void execute() {

			}
		});

		this.display.getEditMenuBar().addItem("Edit Jobs", new Command() {
			public void execute() {

			}
		});

		// menu bar

		this.display.getMenuBar().addItem("Simulation",
				this.display.getSimulationMenuBar());
		this.display.getMenuBar().addSeparator();
		this.display.getMenuBar()
				.addItem("Edit", this.display.getEditMenuBar());
	}

	public Szenario getActualSzenario() {
		return this.currentSzenario;
	}

	public ServiceAsync getService() {
		return this.rpcService;
	}

}
