package uni.oldenburg.client.view;

import java.util.ArrayList;

import uni.oldenburg.client.presenter.MainFramePresenter;
import uni.oldenburg.shared.model.SzenarioInfo;

import com.google.gwt.user.client.ui.Label;
import com.google.gwt.event.dom.client.ChangeEvent;
import com.google.gwt.event.dom.client.ChangeHandler;
import com.google.gwt.event.dom.client.ClickEvent;
import com.google.gwt.event.dom.client.ClickHandler;
import com.google.gwt.user.client.ui.Button;
import com.google.gwt.user.client.ui.DialogBox;
import com.google.gwt.user.client.ui.HasHorizontalAlignment;
import com.google.gwt.user.client.ui.HorizontalPanel;
import com.google.gwt.user.client.ui.ListBox;
import com.google.gwt.user.client.ui.VerticalPanel;


/**
 * Dialogbox for selecting and loading of a Scenario
 * 
 * @author Raschid
 */
public class DialogBoxScenarioSelection extends DialogBox {
	private ArrayList<SzenarioInfo> lstInfos;
	private MainFramePresenter presenter;
	private ListBox lBox;
	VerticalPanel vpInfos = new VerticalPanel();
	
	//The Dialog needs the reference to the Presenter in order to load it
	public DialogBoxScenarioSelection(ArrayList<SzenarioInfo> infos, MainFramePresenter presenter) {
		this.lstInfos=infos;
		this.presenter=presenter;
		this.lBox=new ListBox();
		lBox.setWidth("100%");
		
		// Set the dialog box's caption.
		setText("Choose a Scenario");

		// Enable animation.
		setAnimationEnabled(true);

		// Enabl glass background.
		setGlassEnabled(true);
		
		VerticalPanel vpSzenario = new VerticalPanel();
		vpSzenario.setHorizontalAlignment(HasHorizontalAlignment.ALIGN_CENTER);

		// DialogBox is a SimplePanel, so you have to set its widget property to
		// whatever you want its contents to be.
		
		//Create Button for each Scenario
		for (SzenarioInfo myInfo : lstInfos) {
			lBox.addItem(myInfo.Title);
		}
		
		// initial data
		if (lstInfos.size() > 0)
			setInfoLabelData(lstInfos.get(0));
		
		lBox.addChangeHandler(new ChangeHandler() {
			public void onChange(ChangeEvent event) {
				// set infobox data
				setInfoLabelData(lstInfos.get(lBox.getSelectedIndex()));
			}
		});
		
		
		//Listbox hinzufuegen
		vpSzenario.add(lBox);
		
		vpSzenario.add(vpInfos);
		
		HorizontalPanel hpButtons = new HorizontalPanel();
		hpButtons.setSpacing(5);
		//loads the Szenario by  getting the name of the selected Listboxitem and delivering it to the loadSzenario Method
		//of Mainframepresenter
		Button btnLoad = new Button("Load");
		btnLoad.addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				DialogBoxScenarioSelection.this.presenter.loadSzenario(DialogBoxScenarioSelection.this.lBox.getItemText(DialogBoxScenarioSelection.this.lBox.getSelectedIndex()));
				DialogBoxScenarioSelection.this.hide();
			}
		});
		btnLoad.setWidth("80px");
		
		hpButtons.add(btnLoad);
		
		Button btnExit = new Button("Exit");
		btnExit.addClickHandler(new ClickHandler() {
			public void onClick(ClickEvent event) {
				DialogBoxScenarioSelection.this.hide();
			}
		});
		
		btnExit.setWidth("80px");
		hpButtons.add(btnExit);
		
		vpSzenario.add(hpButtons);
		
		this.setWidget(vpSzenario);
		this.center();
	}
	
	public MainFramePresenter getPresenter() {
		return presenter;
	}
	
	private void setInfoLabelData(SzenarioInfo myInfo) {
		Label lblSeperator = new Label(" ");
		lblSeperator.setHeight("10px");
		
		vpInfos.clear();
		vpInfos.add(new Label("Ramps:"));
		vpInfos.add(createComboLabel("Entry-Ramp Count:", "" + myInfo.EntryRampCount, true));
		vpInfos.add(createComboLabel("Exit-Ramp Count:", "" + myInfo.ExitRampCount, true));
		vpInfos.add(createComboLabel("Storage-Ramp Count:", "" + myInfo.StorageRampCount, true));
		
		int rampCount = myInfo.EntryRampCount + myInfo.ExitRampCount + myInfo.StorageRampCount;
		vpInfos.add(createComboLabel("Complete Ramp Count:", "" + rampCount, true));
		
		vpInfos.add(lblSeperator);
		vpInfos.add(createComboLabel("Vehicle Count:", "" + myInfo.VehicleCount, false));
	}
	
	private HorizontalPanel createComboLabel(String textLeft, String textRight, boolean indented) {
		HorizontalPanel hpLabels = new HorizontalPanel();
		
		// label for indentation
		if (indented == true) {
			Label lblIndent = new Label("");
			lblIndent.setWidth("20px");
			hpLabels.add(lblIndent);		
		}
		
		// left label
		Label lblLeft = new Label(textLeft);
		if (indented == true)
			lblLeft.setWidth("180px");
		else
			lblLeft.setWidth("200px");
		hpLabels.add(lblLeft);		
		
		// right label
		Label lblRight = new Label(textRight);
		lblRight.setWidth("40px");
		lblRight.setHorizontalAlignment(HasHorizontalAlignment.ALIGN_RIGHT);
		hpLabels.add(lblRight);
		
		return hpLabels;
	}
}
