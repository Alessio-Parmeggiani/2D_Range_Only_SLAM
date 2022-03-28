
#lateration algorithm to estimate position fo landmark
#is a range based triangulation algorithm
#different implementation possible, 
#an implementation can be found here http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.7.933&rep=rep1&type=pdf
#at page 504

function [p1,p2]=estimateLandmarkPosition(measurements,poses)
    
	num_measure=length(measurements);

	obs_pose_id=measurements(num_measure).obs_pose_id;
	last_pose=poseFromId(obs_pose_id,poses);
    last_x = last_pose.x;
	last_y = last_pose.y;
	last_r = measurements(num_measure).obs_range;

    A = zeros(num_measure-1, 2);
    b = zeros(num_measure-1, 1);

    for k=1:num_measure-1

		obs_pose_id=measurements(k).obs_pose_id;
		pose=poseFromId(obs_pose_id,poses);
		x = pose.x;
		y = pose.y;
		r = measurements(k).obs_range;
        
        A(k,:) = 2 * [(x - last_x) (y - last_y)];
        b(k,:) = [x^2-last_x^2 + y^2-last_y^2 + last_r^2-r^2];
    endfor

    landmark_position = (A'*A) \ A'*b;

    p1=landmark_position(1);
    p2=landmark_position(2);

end
