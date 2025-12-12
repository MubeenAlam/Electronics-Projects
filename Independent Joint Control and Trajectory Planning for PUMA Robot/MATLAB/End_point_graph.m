% angles from the simulink 
% ang1,ang2 and ang3 are robot joint ange from the simulink
% we are using the forward kinmatics equation to get the position of end
% effector for respective joint angel.
end_effector_pos = double(vpa(subs(forward_kinmatics,{thi1,thi2,thi3},{ang1'-(pi/2),ang2',ang3'-(pi/2)}),3));
X = end_effector_pos(1,:); Y = end_effector_pos(2,:); Z = end_effector_pos(3,:);
%ploting the X,Y,Z coordinates of the end effector.
plot3(X,Y,Z);
grid on
hold on
% marking the starting point.
plot3(X(1),Y(1),Z(1),'o','MarkerSize',10)
% marking the refrence frame origen
plot3(0,0,0,'*r','MarkerSize',15)
grid on
xlabel X
ylabel y
zlabel z
hold off