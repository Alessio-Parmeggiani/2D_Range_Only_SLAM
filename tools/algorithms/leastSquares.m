global pose_dim = 3; 
global landmark_dim = 2;

function [XR,XL]=leastSquares(land_observations,robot_poses,landmark_positions,poses,gt_landmarks,params,land_id_to_idx)
    global pose_dim;
    global landmark_dim;
    #parameters
    kernel_threshold=params.kernel_threshold;
    num_iterations=params.num_iterations;
    damping=params.damping;

    #system parameters
    num_landmarks=length(land_observations);
    num_poses=length(robot_poses);
    system_size=pose_dim*num_poses+landmark_dim*num_landmarks; 
    
    chi_stats   = zeros(1, num_iterations);
    num_inliers = zeros(1, num_iterations);

    
    XR=robot_poses;
    XL=land_observations;
    first_pose_id=params.first_pose_id;
    #I tried to use an association matrix only to check if the order of the measurements
    #changed something
    #A=get_association_matrix(land_observations,num_poses,params);
    #to be used using:         
        #{
        # for each measurement
        for p_idx=1:length(A)
            for l_obs=1:length(land_observations)
                Xr=XR(:,:,p_idx);
                Xl=XL(l_obs).pos;
                z=A(p_idx,l_obs);
        #}
    #

    for iteration=1:num_iterations
        H=zeros(system_size, system_size);
        b=zeros(system_size,1);
        chi_stats(iteration) = 0;

        #this two loops iterate through all the measurements
        for l_obs=1:length(land_observations)
            land_obs=land_observations(l_obs);
            if l_obs<=length(XL)
                for m=1:length(land_obs.measurements)

                    #get needed data for this measurement
                    measurement=land_obs.measurements(m);
                    
                    #pose data
                    pose_id=measurement.obs_pose_id;
                    p_idx=pose_id-first_pose_id+1;
                    Xr=XR(:,:,p_idx);

                    #landmark data
                    l_idx=land_id_to_idx(land_obs.l_id);
                    Xl=XL(l_idx).pos;
                    
                    #range measured
                    z=measurement.obs_range;  
       
                if Xl && z>0
                    #compute error and jacobian
                    [e,Jr,Jl] = errorAndJacobian(Xr, Xl, z);
                    chi=e'*e;
                    if (chi>kernel_threshold)
                        e*=sqrt(kernel_threshold/chi);
                        chi=kernel_threshold;
                    else
                        num_inliers(iteration)++;
                    endif
                    chi_stats(iteration) += chi;

                    #Update H and B
                    pose_matrix_idx=poseMatrixIndex(p_idx, num_poses, num_landmarks);
                    landmark_matrix_idx=landmarkMatrixIndex(l_idx, num_poses, num_landmarks);
                    
                    H(pose_matrix_idx:pose_matrix_idx+pose_dim-1,
                    pose_matrix_idx:pose_matrix_idx+pose_dim-1) += (Jr'*Jr);

                    H(pose_matrix_idx:pose_matrix_idx+pose_dim-1,
                    landmark_matrix_idx:landmark_matrix_idx+landmark_dim-1) += (Jr'*Jl);

                    H(landmark_matrix_idx:landmark_matrix_idx+landmark_dim-1,
                    landmark_matrix_idx:landmark_matrix_idx+landmark_dim-1) += (Jl'*Jl);

                    H(landmark_matrix_idx:landmark_matrix_idx+landmark_dim-1,
                    pose_matrix_idx:pose_matrix_idx+pose_dim-1) += (Jr'*Jl)';

                    b(pose_matrix_idx:pose_matrix_idx+pose_dim-1) += (Jr'*e);
                    b(landmark_matrix_idx:landmark_matrix_idx+landmark_dim-1) += (Jl'*e);
                endif
                endfor
            endif
        endfor


        
        H += eye(system_size)*damping;
        dx = zeros(system_size, 1);

        #solve linear system
        dx(pose_dim+1:end) = -(H(pose_dim+1:end, pose_dim+1:end) \ b(pose_dim+1:end, 1));
        #apply found perturbation
        [XR, XL] = boxPlus(XR, XL, num_poses, num_landmarks, dx);
        
        #print error update
        if mod(iteration,5)==0
            printf("error in landmark position after %d iterations is %f\n",iteration,evaluate(gt_landmarks,XL,land_id_to_idx));
        endif
        
    endfor

endfunction

