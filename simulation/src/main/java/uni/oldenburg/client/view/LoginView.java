package uni.oldenburg.client.view;




import uni.oldenburg.client.presenter.LoginPresenter;

import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.user.client.ui.Button;
import com.google.gwt.user.client.ui.Composite;
import com.google.gwt.user.client.ui.HasValue;
import com.google.gwt.user.client.ui.Label;
import com.google.gwt.user.client.ui.PasswordTextBox;
import com.google.gwt.user.client.ui.TextBox;
import com.google.gwt.user.client.ui.VerticalPanel;
import com.google.gwt.user.client.ui.Widget;

public class LoginView extends Composite implements LoginPresenter.Display {
	
	private TextBox txtbxUsername;
	private PasswordTextBox txtbxPassword;
	private Button loginButton;
	private Button regButton;
  
  public LoginView() {
	    VerticalPanel absolutePanel = new VerticalPanel();
		
		initWidget(absolutePanel);
		
		
		
		txtbxUsername = new TextBox();
		txtbxUsername.setText("Username");
		txtbxUsername.setPixelSize(200, 40);
		absolutePanel.add(txtbxUsername);
		
	    txtbxPassword = new PasswordTextBox();
		txtbxPassword.setText("Password");
		txtbxPassword.setPixelSize(200, 40);
		absolutePanel.add(txtbxPassword);
		
		
		loginButton = new Button("Senden");
		loginButton.setText("Login");
		loginButton.setSize("100px", "30px");
		absolutePanel.add(loginButton);
		
		
		regButton = new Button("Registrieren");
		regButton.setText("Registrieren");
		regButton.setSize("100px", "30px");
		absolutePanel.add(regButton);
  }
  
  
  public HasValue<String> getTextboxUsername(){
		return txtbxUsername;
	}
	
	public HasValue<String> getTextboxPassword(){
		return txtbxPassword;
	}
	
	public HasClickHandlers getLoginButton(){
		return loginButton;
	}
	
	public HasClickHandlers getRegistrierenButton(){
		return regButton;
	}
	
	public Widget asWidget(){
		return this;
	}
  
  
}
