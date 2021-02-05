%%
base_link = Link('d', 101.115e-3, 'a', 0, 'alpha', -pi/2);%Create the 3 links
link_1 = Link('d', 0, 'a', 0, 'alpha', pi/2);
link_2 = Link('d', 333.95e-3, 'a', 0, 'alpha', pi/2);
link_3 = Link('d', 28.62e-3, 'a', 89.2e-3);
Robot = SerialLink([base_link, link_1, link_2, link_3], 'name', 'Arm');%Add the 4 links to a robot
q0 = [0.5,0.5,0.5,0.5];
q = q0;
Robot.plot(q0);
%%
loop = true;
dt = 0.1;
x_desired = 0.2;
y_desired = -0.1;
z_desired = 0.2;
roll_desired = 0.4;
pitch_desired = 0.15;
yaw_desired = 0;

app = gui_single_arm;
init_tr = Robot.fkine(q);
[rot,xyz] = tr2rt(init_tr);
rpy = tr2rpy(rot,'xyz');
app.X_desired.Value = xyz(1);
app.Y_desired.Value = xyz(2);
app.Z_desired.Value = xyz(3);
app.Roll_desired.Value = rpy(1);
app.Pitch_desired.Value = rpy(2);
app.Yaw_desired.Value = rpy(3);
%input('Type in position in GUI, then press enter');
while(loop)
    x_desired = app.X_desired.Value;
    y_desired = app.Y_desired.Value;
    z_desired = app.Z_desired.Value;
    roll_desired = app.Roll_desired.Value;
    pitch_desired = app.Pitch_desired.Value;
    yaw_desired = app.Yaw_desired.Value;
    Robot.plot(q);%Plot the robot at the given joint angles
    T = Robot.fkine(q);%Forward kinematics
    rpy = tr2rpy(T,'xyz');
    [~,xyz] = tr2rt(T);
    app.X.Text = num2str(xyz(1), 3);
    app.Y.Text = num2str(xyz(2), 3);
    app.Z.Text = num2str(xyz(3), 3);
    app.Roll.Text = num2str(rpy(1), 3);
    app.Pitch.Text = num2str(rpy(2), 3);
    app.Yaw.Text = num2str(rpy(3), 3);
    JA = Robot.jacob0(q,'rpy');%Analytical Jacobian in xyz euler angles
    j_inv = pinv(JA);
    x_error = 5*(x_desired - xyz(1));
    y_error = 5*(y_desired - xyz(2));
    z_error = 5*(z_desired - xyz(3));
    roll_error = 0%roll_desired - rpy(1);
    pitch_error = 0%pitch_desired - rpy(2);
    yaw_error = 0%yaw_desired - rpy(3);
    errors = [x_error, y_error, z_error, roll_error, pitch_error, yaw_error];
    if any(abs(errors./5)>1e-3)
        q_dot = j_inv*[x_error; y_error; z_error; roll_error; pitch_error; yaw_error];
        q = (q + (q_dot.*dt)');
    end
    pause(0.1);
end

