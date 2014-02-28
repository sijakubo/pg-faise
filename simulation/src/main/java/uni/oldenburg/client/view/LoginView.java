package uni.oldenburg.client.view;

import com.google.gwt.event.dom.client.HasClickHandlers;
import com.google.gwt.user.client.ui.*;
import uni.oldenburg.client.presenter.LoginPresenter;

public class LoginView extends Composite implements LoginPresenter.IDisplay {
    private TextBox txtEmail;
    private PasswordTextBox txtPassword;
    private Button btnLogin;
    private Button btnRegister;

    public LoginView() {
        Grid grid = new Grid(3, 2);
        grid.setWidget(0, 0, new Label("E-Mail: "));
        grid.setWidget(0, 1, createEmailInput());

        grid.setWidget(1, 0, new Label("Passwort: "));
        grid.setWidget(1, 1, createPasswordInput());

        grid.setWidget(2, 0, createLoginButton());
        grid.setWidget(2, 1, createButtonRegister());

        VerticalPanel vpLogin = new VerticalPanel();
        vpLogin.add(grid);
        initWidget(vpLogin);
    }

    private Button createButtonRegister() {
        btnRegister = new Button("Registrieren");
        btnRegister.setText("Registrieren");
        btnRegister.setSize("100px", "30px");
        return btnRegister;
    }

    private Button createLoginButton() {
        btnLogin = new Button("Login");
        btnLogin.setText("Login");
        btnLogin.setSize("100px", "30px");
        return btnLogin;
    }

    private IsWidget createPasswordInput() {
        txtPassword = new PasswordTextBox();
        txtPassword.setPixelSize(200, 30);
        return txtPassword;
    }

    private IsWidget createEmailInput() {
        txtEmail = new TextBox();
        txtEmail.setPixelSize(200, 30);
        return txtEmail;
    }


    public HasValue<String> getEmail() {
        return txtEmail;
    }

    public HasValue<String> getPassword() {
        return txtPassword;
    }

    public HasClickHandlers getLoginButton() {
        return btnLogin;
    }

    public HasClickHandlers getRegisterButton() {
        return btnRegister;
    }
}
