drone_profile_data



x = drone_profile(:,1);
y = drone_profile(:,2);
z = drone_profile(:,3);

%y_data = -y(6:16);
y_data = -y;

T = 1/30;

t_data = [0: length(y_data)-1] * T;

figure(1)

plot(t_data, y_data)

figure(2)
plot(x)
hold on
plot(y)
hold on
plot(z)
hold off
grid on

thrust_profile_data

figure(3)
xx = thrust_array(:,1);
yy = thrust_array(:,2);
zz = thrust_array(:,3);
plot(xx)
hold on
plot(x)

plot(yy)
plot(y)

plot(zz)
plot(z)

hold off

figure(4)
subplot(2,1,1)
ux = thrust_array(:,3);
plot(ux);
subplot(2,1,2)
uz = thrust_array(:,4);
plot(uz);


figure(5)
x_hat = thrust_array(:,1);
plot(x_hat);
hold on
plot(x);
z_hat = thrust_array(:,2);
plot(z_hat);
plot(z);
hold off