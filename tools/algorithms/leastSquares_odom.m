global pose_dim = 3; 
global landmark_dim = 2;

function [XR,XL]=leastSquares_odom(land_observations,robot_poses,landmark_positions,poses,T,gt_landmarks,gt_poses,params,land_id_to_idx)
    global pose_dim;
    global landmark_dim;
    %parameters
    kernel_threshold=params.kernel_threshold;
    num_iterations=params.num_iterations;
    damping=params.damping;

    %system parameters
    num_landmarks=length(land_observations);
    num_poses=length(robot_poses);
    system_size=pose_dim*num_poses+landmark_dim*num_landmarks; 
    
    chi_stats   = zeros(1, num_iterations);
    chi_stats_p   = zeros(1, num_iterations);
    num_inliers = zeros(1, num_iterations);

    
    XR=robot_poses;
    XL=land_observations;
    first_pose_id=params.first_pose_id;

    A=get_association_matrix(land_observations,num_poses,params);       
    
    for iteration=1:num_iterations
        H=zeros(system_size, system_size);
        b=zeros(system_size,1);
        chi_stats(iteration) = 0;
        chi_stats_p(iteration)=0;

        %odometry measurement
        for t=1:length(T)  
            transition=T(t);

            p_idx=(transition.id_from - first_pose_id)+1;
            p_idx2=(transition.id_to - first_pose_id)+1;
            prev_pose=XR(:,:,p_idx);
            next_pose=XR(:,:,p_idx2);
            v=transition.v;
            [ep,J1,J2] = errorJacobianOdometry(prev_pose,next_pose,v);

            chi=ep'*ep;
            if (chi>kernel_threshold)
                ep*=sqrt(kernel_threshold/chi);
                chi=kernel_threshold;
            else
                num_inliers(iteration)++;
            endif
            chi_stats_p(iteration) += chi;

            %Update H and B
            
            p_m_1=poseMatrixIndex(p_idx, num_poses, num_landmarks);
            p_m_2=poseMatrixIndex(p_idx2, num_poses, num_landmarks);

            H(p_m_1:p_m_1+pose_dim-1,p_m_1:p_m_1+pose_dim-1) += J1'*J1;
            H(p_m_1:p_m_1+pose_dim-1,p_m_2:p_m_2+pose_dim-1) += J1'*J2;
            H(p_m_2:p_m_2+pose_dim-1,p_m_1:p_m_1+pose_dim-1) += (J1'*J2)';
            H(p_m_2:p_m_2+pose_dim-1,p_m_2:p_m_2+pose_dim-1) += J2'*J2;

            b(p_m_1:p_m_1 + pose_dim-1) += J1'*ep;
            b(p_m_2:p_m_2+pose_dim-1) += J2'*ep;
            
        endfor

        %range measurements
        %these two loops iterate through all the measurements
        for p_idx=1:length(A)
            for l_idx=1:length(land_observations)
                Xr=XR(:,:,p_idx);
                Xl=XL(l_idx).pos;
                z=A(p_idx,l_idx);

                if Xl && z>0
                    %compute error and jacobian
                    [e,Jr,Jl] = errorAndJacobian(Xr, Xl, z);
                    chi=e'*e;
                    if (chi>kernel_threshold)
                        e*=sqrt(kernel_threshold/chi);
                        chi=kernel_threshold;
                    else
                        num_inliers(iteration)++;
                    endif
                    chi_stats(iteration) += chi;

                    %Update H and B
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

        endfor
        
        H += eye(system_size)*damping;
        dx = zeros(system_size, 1);

        %solve linear system
        dx(pose_dim+1:end) = -(H(pose_dim+1:end, pose_dim+1:end) \ b(pose_dim+1:end, 1));
        %apply found perturbation
        [XR, XL] = boxPlus(XR, XL, num_poses, num_landmarks, dx);
        
        %print error update
        if mod(iteration,5)==0
            printf("After %d iterations...\n",iteration)
            printf("\t mean error in landmark position is %f\n",evaluate(gt_landmarks,XL,land_id_to_idx));
            err_pose=evaluatePoses(gt_poses,XR);
            printf("\t mean error in poses is %f\n",err_pose);
        endif

        
    endfor

endfunction

