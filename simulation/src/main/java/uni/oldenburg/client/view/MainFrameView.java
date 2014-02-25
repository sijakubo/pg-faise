package uni.oldenburg.client.view;

import uni.oldenburg.client.presenter.MainFramePresenter;

import com.google.gwt.user.client.Command;
import com.google.gwt.user.client.ui.MenuBar;
import com.google.gwt.user.client.ui.Composite;
import com.google.gwt.user.client.ui.VerticalPanel;

public class MainFrameView extends Composite implements MainFramePresenter.IDisplay {
	
	private MenuBar  menuBar;
	
	private MenuBar  fileMenuBar;
	private MenuBar  editMenuBar;
	private MenuBar  viewMenuBar;
	
	public MainFrameView() {

		VerticalPanel vpMainFrame = new VerticalPanel();
			
		//file menu
		
		fileMenuBar = new MenuBar(true);
		fileMenuBar.setAnimationEnabled(true);		
		fileMenuBar.addItem("Load Scenario", new Command() {
			public void execute() {
				
			}
		});
		fileMenuBar.addItem("Recently Used", new Command() {
			public void execute() {
				
			}
		});
		fileMenuBar.addSeparator();
		fileMenuBar.addItem("Adjust Scenario", new Command() {
			public void execute() {
				
			}
		});
		fileMenuBar.addItem("Save", new Command() {
			public void execute() {
				
			}
		});
		fileMenuBar.addItem("Save As", new Command() {
			public void execute() {
				
			}
		});
		fileMenuBar.addSeparator();
		fileMenuBar.addItem("Print", new Command() {
			public void execute() {
				
			}
		});
		fileMenuBar.addItem("Exit", new Command() {
			public void execute() {
				
			}
		});

		//edit menu
		
		editMenuBar = new MenuBar(true);
		editMenuBar.setAnimationEnabled(true);
		editMenuBar.addItem("Undo", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addItem("Redo", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addSeparator();
		editMenuBar.addItem("Define Job Set", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addItem("Edit Jobs", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addItem("Change Database", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addItem("Simulation Settings", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addItem("Start/Stop Simulation", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addSeparator();
		editMenuBar.addItem("Save As Image", new Command() {
			public void execute() {
				
			}
		});
		editMenuBar.addItem("PlugIn", new Command() {
			public void execute() {
				
			}
		});

		//view menu
		
		viewMenuBar = new MenuBar(true);
		viewMenuBar.setAnimationEnabled(true);
		viewMenuBar.addItem("Brightness", new Command() {
			public void execute() {
				
			}
		});
		viewMenuBar.addSeparator();
		viewMenuBar.addItem("Zoom In", new Command() {
			public void execute() {
				
			}
		});
		viewMenuBar.addItem("Zoom Out", new Command() {
			public void execute() {
				
			}
		});
		viewMenuBar.addSeparator();
		viewMenuBar.addItem("Fullscreen", new Command() {
			public void execute() {
				
			}
		});
		
		//menu bar
		
		menuBar = new MenuBar();
		menuBar.addItem("File", fileMenuBar);
		menuBar.addSeparator();
		menuBar.addItem("Edit", editMenuBar);
		menuBar.addSeparator();
		menuBar.addItem("View", viewMenuBar);
		menuBar.addSeparator();
		menuBar.addItem("Help", new Command() {
			public void execute() {
				
			}
		});
		vpMainFrame.add(menuBar);
		
		initWidget(vpMainFrame);
	}
}