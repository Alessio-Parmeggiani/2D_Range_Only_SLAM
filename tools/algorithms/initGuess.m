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
    %{
    %compute trajectory using poses
    robot_poses=zeros(3, 3, length(poses));
    for i=1:length(poses)
        robot_poses(:,:,i)=v2t([poses(i).x poses(i).y poses(i).theta]);
    endfor
    %}

    %compute trajectory using transitions
    robot_poses=zeros(3, 3, length(poses));
    robot_poses(:,:,1)=v2t([poses(1).x poses(1).y poses(1).theta]);
    prev_pose=robot_poses(:,:,1);
    for i=1:length(transitions)
        v=transitions(i).v;
        displacement=v2t([v(1) v(2) v(3)]);
        next_pose=prev_pose*displacement;
        prev_pose=next_pose;
        robot_poses(:,:,i+1)=next_pose;
    endfor


    
endfunction





