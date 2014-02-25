package uni.oldenburg.client.view;

import uni.oldenburg.client.presenter.MainFramePresenter;

import com.google.gwt.user.client.Command;
import com.google.gwt.user.client.ui.MenuBar;
import com.google.gwt.user.client.ui.Composite;
import com.google.gwt.user.client.ui.VerticalPanel;

public class MainFrameView extends Composite implements MainFramePresenter.IDisplay {
	
	private MenuBar menuBar;
	
	private MenuBar fileMenu;
	private MenuBar editMenu;
	private MenuBar viewMenu;
	
	public MainFrameView() {

		VerticalPanel vpLogin = new VerticalPanel();
			
		//file menu
		
		fileMenu = new MenuBar(true);
		fileMenu.setAnimationEnabled(true);
		
		fileMenu.addItem("New", new Command() {
			public void execute() {
				
			}
		});
		fileMenu.addItem("Open", new Command() {
			public void execute() {
				
			}
		});
		fileMenu.addItem("Save", new Command() {
			public void execute() {
				
			}
		});
		fileMenu.addSeparator();
		fileMenu.addItem("Exit", new Command() {
			public void execute() {
				
			}
		});

		//edit menu
		
		editMenu = new MenuBar(true);
		editMenu.setAnimationEnabled(true);

		//view menu
		
		viewMenu = new MenuBar(true);
		viewMenu.setAnimationEnabled(true);
		
		//menu bar
		
		menuBar = new MenuBar();
		menuBar.addItem("File", fileMenu);
		menuBar.addSeparator();
		menuBar.addItem("Edit", editMenu);
		menuBar.addSeparator();
		menuBar.addItem("View", viewMenu);
		vpLogin.add(menuBar);
		
		initWidget(vpLogin);
	}
}