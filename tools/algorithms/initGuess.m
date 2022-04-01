source 'tools/utilities/geometry_helpers_2d.m'

function [landmark_positions,robot_poses,land_observations,l_id_to_idx] = initGuess(observations,poses,transitions,params)
    %array of landmark_observations, each one correspond to a landmark
    %each landmark_observations is an array of measures (pose id,range)
    land_observations=[];

    first_pose_id=params.first_pose_id;
    
    %used to transform landmark id into indeces 1...n
    %use big number because I don't know the biggest index I could have
    l_id_to_idx = ones(10000, 1)*-1;

    %index for the landmarks
    l_idx=1;

    %building a structure that contains for each landmark all the measurement related to it.
    for i=1:length(observations)
        pose_id=observations(i).pose_id;
        landmarks_observed=observations(i).observation;
        for j=1:length(landmarks_observed)
            observation=landmarks_observed(j);
            
            %pose can be retrieved by the poses array, index is pose_id-first_pose_id+1
            %obs_pose=poses(pose_id-first_pose_id+1);
            
            %i want the landmarks to be stored independently from id but accessed fast
            l_id=observation.id;
            if l_id_to_idx(l_id)==-1
                l_id_to_idx(l_id)=l_idx;
                land_observations(l_id_to_idx(l_id)).l_id=l_id;
                l_idx=l_idx+1;
            endif

            %adding this measurement
            land_observations(l_id_to_idx(l_id)).measurements(end+1).obs_pose_id=pose_id;
            land_observations(l_id_to_idx(l_id)).measurements(end).obs_range=observation.range;
        endfor
    endfor


    
    %for each landmark with 3 or more measurements estimate position
    for i=1:length(land_observations)
        measurements=land_observations(i).measurements;
        
        if length(measurements)>=3
            [px py]=estimateLandmarkPosition(measurements,poses);
            land_observations(i).pos=[px; py];
            landmark_positions(end+1).id=i;
            landmark_positions(end).pos=[px; py];
        endif

    endfor


    %get initial poses in matrix form
    
    % no real difference in using simply the poses or computing the trajectory using the transitions
    
    %compute trajectory using poses
    robot_poses=zeros(3, 3, length(poses));
    for i=1:length(poses)
        robot_poses(:,:,i)=v2t([poses(i).x poses(i).y poses(i).theta]');
    endfor
    
    
    %compute trajectory using transitions
    robot_poses_odom=zeros(3, 3, length(poses));
    robot_poses_odom(:,:,1)=v2t([poses(1).x poses(1).y poses(1).theta]);
    prev_pose=robot_poses_odom(:,:,1);
    for i=1:length(transitions)
        V=transitions(i).v;
        displacement=v2t([V(1) V(2) V(3)]);


        prev_pose=t2v(prev_pose);
        v=V(1);
        theta=prev_pose(3);
        next_pose=[prev_pose(1)+ v *cos(theta);
                    prev_pose(2)+ v *sin(theta);
                    theta+V(3) ];
        next_pose=v2t(next_pose);
        

        #next_pose=displacement*prev_pose;
        prev_pose=next_pose;
        robot_poses_odom(:,:,i+1)=next_pose;
    endfor

    printf("Looking for differences between pose and pose using odometry measurements\n")
    total_dif=0;
    max_dif=-1;
    for i=1:length(robot_poses)
        p1=t2v(robot_poses(:,:,i));
        p2=t2v(robot_poses_odom(:,:,i));
        dif=norm(p1-p2);
        total_dif+=dif;
        if dif>max_dif
            max_dif=dif;
        endif
    endfor
    mean_dif=total_dif/length(robot_poses);
    printf("\tMaximum difference is %f\n\tmean of differences is %f\n\n",max_dif,mean_dif)
    
    #myDrawTrajectory(robot_poses,robot_poses,robot_poses2,length(robot_poses))

    robot_poses=robot_poses_odom;

    
endfunction





