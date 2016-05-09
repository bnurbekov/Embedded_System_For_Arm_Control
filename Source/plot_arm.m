function objects = plot_arm(joint_angle_1, joint_angle_2, link_length_1, link_length_2)
%plot_arm(joint_angle1, joint_angle2, link_length_1, link_length_2)
%  joint_angle_1:  the angle of the first joint
%  joint_angle_2:  the angle of the second joint
%  link_length_1:  the length of the first link
%  link_length_2:  the length of the second link

    link_length_0 = 6;
    a1 = link_length_1;
    a2 = link_length_2;
    theta1 = joint_angle_1 * pi / 180; %convert degrees to radians
    theta2 = joint_angle_2 * pi / 180; %convert degrees to radians
    joint_position_1 = [a1*cos(theta1) a1*sin(theta1)];
    joint_position_2 = [a1*cos(theta1)+a2*cos(theta1+theta2)  a1*sin(theta1)+a2*sin(theta1+theta2)];
    
    horizontal_link = line([-3 0], [-link_length_0 -link_length_0], 'LineWidth', 5, 'Color', 'blue');
    link0 = line([0 0], [0 -link_length_0], 'LineWidth', 5, 'Color', 'blue'); 
    link1 = line([0 joint_position_1(1)], [0 joint_position_1(2)], 'LineWidth', 3, 'Color', 'red'); 
    link2 = line([joint_position_1(1) joint_position_2(1)], [joint_position_1(2) joint_position_2(2)], 'LineWidth', 3, 'Color', 'red');
    joint0 =  viscircles([0 0], 0.1, 'LineWidth', 2, 'EdgeColor', 'black');
    joint1 =  viscircles([joint_position_1(1) joint_position_1(2)], 0.1, 'LineWidth', 2, 'EdgeColor', 'black'); 
    joint2 =  viscircles([joint_position_2(1) joint_position_2(2)], 0.1, 'LineWidth', 2, 'EdgeColor', 'black');
 
    %scale the view equally on vertical and horizontal lines
    axis equal;
    
    objects = [horizontal_link, link0, joint0, link1, joint1, link2, joint2];
end
