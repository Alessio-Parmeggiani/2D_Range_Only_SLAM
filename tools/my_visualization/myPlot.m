1;
function myPlot(init_poses,optim_poses,gt_poses,init_land,optim_land,gt_land)
	myDrawLandmarks(init_land,optim_land,gt_land,false);
	myDrawTrajectory(init_poses,optim_poses,gt_poses);
	
endfunction

%plotting trajectories
function myDrawTrajectory(init,optim,gt,poses_num)
	%I have all trajectories in matrix form
	poses_num=length(gt);
    figure();
    hold on;
    title("Robot trajectory ");
    
    plot(reshape(init(1,3,:), 1, poses_num), reshape(init(2,3,:), 1, poses_num), 'b', 'linewidth', 2);
    plot(reshape(optim(1,3,:), 1, poses_num), reshape(optim(2,3,:), 1, poses_num), 'g', 'linewidth', 2);
    plot(reshape(gt(1,3,:), 1, poses_num), reshape(gt(2,3,:), 1, poses_num), 'k', 'linewidth', 1);
    legend("Initial guess","after Least Squares","Ground Truth");	
    print(fullfile("trajectory_plot.png"));
	hold off;
end

%plotting landmarks 
%cosnidering outliers, there is a landmark that should be outside of the plot
%otherwise it looks stretched
function done = myDrawLandmarks(init,optim,truth,draw_labels=false)
	%first put all landmarks in a format to plot in one line
	% in this way I can plot also a legend

	% I use 3 loops to avoid any cases of errors with different sizes of the landamrks.
	% it could happen if for example I have one landmark that has not been observed by any pose
	init_land=zeros(2,length(init));
	valid_index=1;
	for l=1:length(init)
		if init(l).pos && init(l).pos(2)<15
			init_land(:,end+1)=init(l).pos;
			valid_index=l;
		else 
			%this landmark position has not been estimated, use random one for plot
			init_land(:,end+1)=init(valid_index).pos;
			%printf("null landmark on init guess\n")
		endif
	endfor

	optim_land=zeros(2,length(optim));
	for l=1:length(optim)
		if optim(l).pos
			optim_land(:,end+1)=optim(l).pos;
			valid_index=l;
		else 
			optim_land(:,end+1)=optim(valid_index).pos;
			%printf("optim invalid landmark\n");
		endif
	endfor

	truth_land=zeros(2,length(truth));
	for l=1:length(truth)
		truth_land(:,end+1)=[truth(l).x_pose; truth(l).y_pose];
	endfor
	
	if(nargin == 2)
		color = 'r';
		mode = 'fill';
	end
	hold on;

	title("Landmark positions");
	scale=20;
	color="k";
	scatter(truth_land(1,:),truth_land(2,:),scale,color,"filled");
	color="b";
	scatter(init_land(1,:),init_land(2,:),scale,color,"filled");
	color="g";
	scatter(optim_land(1,:),optim_land(2,:),scale,color,"filled");
	legend("Ground Truth", "initial Guess ", "Least Square optimization")
	hold off;
	print(fullfile("landmark_plot_no_outlier.png"));
	
	
end



function drawLandmarkWithLabel(land,truth,radius=0.3)
	hold on;
	
	color="g";
	for i=1:length(land)
		if land(i).pos
			drawShape('circle', [land(i).pos(1) land(i).pos(2), radius], 'fill', color);
			drawLabels(land(i).pos(1), land(i).pos(2), land(i).l_id, '%d');
		endif
	endfor
	color="k";
	for i=1:length(truth)
		drawShape('circle', [truth(i).x_pose truth(i).y_pose, radius], 'fill', color);
		drawLabels(truth(i).x_pose, truth(i).y_pose, truth(i).id, '%d');
	endfor
	print(fullfile("landmark_plot_labels.png"));
	hold off;
endfunction

