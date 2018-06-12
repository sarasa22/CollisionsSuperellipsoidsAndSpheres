clc;
clear;


%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Parameters %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%

%__________Robot's Body or Obstacles

%parameter e: for robot's body (elliptic cylinder) -> e1 = 0.1 e e2 = 1
%             for obstacles (ellipsoid) -> e1 = 1 e e2 = 1
e1 = 0.1; e2 = 1;

%size
a1 = 680.0/2; a2 = 770.0/2; a3 = 920.0/2;

%position
p_x = 49.984; p_y = 184.987; p_z = 450.003;

%orientation
roll = 0.00000733138; pitch = 0.000000891009; yaw = -3.14159;



%__________Manipulator's arm points

%parameter e: sphere -> e1 = 1 e e2 = 1
pointArm_e1 = 1; point_e2 = 1;

%sphere size
sphereArm_rad = 150;
pointArm_a1 = sphereArm_rad; pointArm_a2 = sphereArm_rad; pointArm_a3 = sphereArm_rad;

%position
pointArm_p_x = 79.984; pointArm_p_y = 949.987; pointArm_p_z = 1090.003;

%orientation
pointArm_roll = -3.14159; pointArm_pitch = 0.00000733138; pointArm_yaw = 0.00000733138;





%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% Representations %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%


%Representation of the robot's body or obstacles
j = 1;
for beta = -pi : pi/10 : pi
    i = 1;
    for theta = -pi/2 : pi/20 : pi/2
        xA(i,j)= (a1) * sign(cos(theta)) * abs(cos(theta)).^e1 * sign(cos(beta)) * abs(cos(beta)).^e2;
        yA(i,j)= (a2) * sign(cos(theta)) * abs(cos(theta)).^e1 * sign(sin(beta)) * abs(sin(beta)).^e2;
        zA(i,j)= (a3) * sign(sin(theta)) * abs(sin(theta)).^e1;
        i = i + 1;
    end
    j = j + 1;
end


%Representation of the manipulator's arm points
k = 1;
for beta = -pi : pi/10 : pi
    l = 1;
    for theta = -pi/2 : pi/20 : pi/2
        xB(l,k)= pointArm_a1 * sign(cos(theta)) * abs(cos(theta)).^pointArm_e1 * sign(cos(beta)) * abs(cos(beta)).^point_e2;
        yB(l,k)= pointArm_a2 * sign(cos(theta)) * abs(cos(theta)).^pointArm_e1 * sign(sin(beta)) * abs(sin(beta)).^point_e2;
        zB(l,k)= pointArm_a3 * sign(sin(theta)) * abs(sin(theta)).^pointArm_e1;
        l = l + 1;
    end
    k = k + 1;
end


%Rotation of the robot's body or obstacles
Rot = RPY(roll, pitch, yaw);
%Rotation of manipulator's arm points
pointArm_Rot = RPY(pointArm_roll, pointArm_pitch, pointArm_yaw);


%Global coordinates of the robot's body or obstacles
xA_global= p_x + xA*Rot(1,1) + yA*Rot(1,2) + zA*Rot(1,3);
yA_global= p_y + xA*Rot(2,1) + yA*Rot(2,2) + zA*Rot(2,3);
zA_global= p_z + xA*Rot(3,1) + yA*Rot(3,2) + zA*Rot(3,3);
%Global coordinates of the robot's body or obstacles
xB_global= pointArm_p_x + xB*pointArm_Rot(1,1) + yB*pointArm_Rot(1,2) + zB*pointArm_Rot(1,3);
yB_global= pointArm_p_y + xB*pointArm_Rot(2,1) + yB*pointArm_Rot(2,2) + zB*pointArm_Rot(2,3);
zB_global= pointArm_p_z + xB*pointArm_Rot(3,1) + yB*pointArm_Rot(3,2) + zB*pointArm_Rot(3,3);


%Create figure
figure1 = figure('PaperSize',[20.98 29.68],'Color',[1 1 1],...
    'NumberTitle','off','ToolBar','figure','MenuBar','none','Units',...
    'pixels');
annotation(gcf,'textbox',[0.5 0.99 0 0],...
    'HorizontalAlignment','center','FontWeight',...
    'bold','FontSize',16,'FontName','Calibri','FitBoxToText','on',...
    'EdgeColor','none');
axes1 = axes('Parent',gcf,'DataAspectRatio',[1 1 1],...
    'Color',[.9 .9 .9],...
    'GridLineStyle','--',...
    'TickDir','out', 'xticklabel', [],'yticklabel', [],'zticklabel', [],...
    'FontName','Calibri','FontWeight','light','FontSize',12);
xlabel('x axis','FontSize',14);
ylabel('y axis','FontSize',14);
zlabel('z axis','FontSize',14);
title(['\fontsize{16}Distance between a superellipsoid and a sphere']); 
set(gcf,'color','white', 'Position',[20 50 1000 750],...
    'nextplot','replacechildren');
view([110 20]);
hold on;
lighting GOURAUD % FLAT, GOURAUD, PHONG, NONE
camlight headlight


%Robot's body model or obstacles model
body = mesh(xA_global,yA_global,zA_global,'Parent',gca,...
    'EdgeLighting','flat',...
    'FaceLighting','flat',...
    'LineWidth',0.5,...
    'FaceColor',[0.80 0.93 1.0],...
    'FaceAlpha',0.3,...
    'EdgeColor','b',...
    'EdgeAlpha', 0.5);
%Manipulator's arm points model
sphere = mesh(xB_global,yB_global,zB_global,'Parent',gca,...
    'EdgeLighting','flat',...
    'FaceLighting','flat',...
    'LineWidth',0.5,...
    'FaceColor',[1 1 0],...
    'FaceAlpha',0.3,...
    'EdgeColor','interp',...
    'EdgeAlpha', 0.5);

lgd = legend(sprintf('Robot body \n (e1 = 0.1 and e2 = 1) \n \n '),...
       sprintf('Manipulator arm points \n (e1 = 1 and e2 = 1) \n \n'), ...
       'Location' , 'east');
title(lgd,'Models')

   
%Surfaces centers
plot3(p_x, p_y, p_z,'.r', 'markersize',70);
plot3(pointArm_p_x, pointArm_p_y, pointArm_p_z,'.r','markersize',50);
%Line between centers of surfaces
P1 = [p_x, p_y, p_z];
P2 = [pointArm_p_x, pointArm_p_y, pointArm_p_z];
pts = [P1; P2];   
plot3(pts(:,1), pts(:,2), pts(:,3), 'LineStyle' , ':', 'Color', [1 0.5 0], 'linewidth', 3);




   
%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% Collisions check %%%%%%%%%%%%%%%%%%%%%%%%%%
%%%

%Check if the center of the sphere lies inside the superellipsoid
f = function_inside_outside(pointArm_p_x, pointArm_p_y, pointArm_p_z, ...
                            p_x, p_y, p_z, a1, a2, a3, Rot, e1, e2);
      
                        
%Computes distance between the center of the sphere and the center of 
%the superellipsoid
ro = ((pointArm_p_x - p_x).^2 + (pointArm_p_y - p_y).^2 + (pointArm_p_z - p_z).^2).^(1/2);


%Computes distance between the surface of the superellipsoid and the
%center of the sphere
d = ro * abs(1 - f.^(-e1/2));


%Check if the sphere lies inside the superellipsoid
cond = ro * (1 - f.^(-e1/2)) - sphereArm_rad;
if  cond >= 0
    msgbox('The point of arm is outside the superellipsoid!','Sucess');
else
    msgbox('The point of arm is inside the superellipsoid!','Error','error');
end


axis equal; axis on; grid on;
