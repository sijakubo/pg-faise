package uni.oldenburg.client.view;

import java.util.ArrayList;

import uni.oldenburg.client.presenter.MainFramePresenter;

import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.user.client.ui.Button;
import com.google.gwt.user.client.ui.DialogBox;
import com.google.gwt.user.client.ui.HasHorizontalAlignment;
import com.google.gwt.user.client.ui.ListBox;
import com.google.gwt.user.client.ui.VerticalPanel;


/**
 * Dialogbox for selecting and loading of a Scenario
 * 
 * @author Raschid
 */
public class DialogBoxJoblistSelection extends DialogBox {
	@SuppressWarnings("unused")
	private ArrayList<String> titles;
	private MainFramePresenter presenter;
	private ListBox lBox;
	
	//The Dialog needs the reference to the Presenter in order to load it
	public DialogBoxJoblistSelection(ArrayList<String> titles, MainFramePresenter presenter) {
		this.titles=titles;
		this.presenter=presenter;
		this.lBox=new ListBox();
		
		// Set the dialog box's caption.
		setText("Choose a Joblist");

		// Enable animation.
		setAnimationEnabled(true);

		// Enabl glass background.
		setGlassEnabled(true);
		
		VerticalPanel vpSzenario = new VerticalPanel();
		vpSzenario.setHorizontalAlignment(HasHorizontalAlignment.ALIGN_RIGHT);
		
		
		

		// DialogBox is a SimplePanel, so you have to set its widget property to
		// whatever you want its contents to be.
		
		//Create Button for each Joblist
		for(int i=0; i<titles.size(); i++){
			lBox.addItem(titles.get(i));
		}
		
		
		
		//Listbox hinzufuegen
		vpSzenario.add(lBox);
				
		//loads the Joblist by  getting the name of the selected Listboxitem and delivering it to the loadJoblist Method
		//of Mainframepresenter
		Button load = new Button("load");
		load.addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				DialogBoxJoblistSelection.this.presenter.loadJoblist(DialogBoxJoblistSelection.this.lBox.getItemText(DialogBoxJoblistSelection.this.lBox.getSelectedIndex()));
				DialogBoxJoblistSelection.this.hide();
			}
		});
		
		vpSzenario.add(load);
		
		
		
		Button exit = new Button("Exit");
		exit.addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				DialogBoxJoblistSelection.this.hide();
			}
		});
		
		vpSzenario.add(exit);
		
		this.setWidget(vpSzenario);
		this.center();
	}
	
	public MainFramePresenter getPresenter() {
		return presenter;
	}
}
