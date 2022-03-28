#size of pose and landmarks
global pose_dim=3;
global landmark_dim=2;

#matrix of measurements
#row:poses;   columns:landmarks;
#so A(pose,landmark) correspond to the measure
#used also to test if a different order of the measurements change the Least squares results
function A=get_association_matrix(land_observations,num_poses,params)
  landmark_num=length(land_observations);
  A = ones(num_poses,landmark_num)*-1;
  first_pose_id=params.first_pose_id;
  
  #for each landmark 
  for l_idx=1:length(land_observations)
    land_obs=land_observations(l_idx);
    #for each measure from the poses
    for m=1:length(land_obs.measurements)
      measurement=land_obs.measurements(m);
      pose_id=measurement.obs_pose_id; 
      pose_idx=pose_id-first_pose_id+1;
      z=measurement.obs_range;
      A(pose_idx,l_idx)=z;
    endfor
  endfor
endfunction

#retrieve index from H matrix
function v_idx=poseMatrixIndex(pose_index, num_poses, num_landmarks)
  global pose_dim;
  global landmark_dim;

  if (pose_index>num_poses)
    printf("error matrix index: pose idx > num_poses\n")
    v_idx=-1;
    return;
  endif;
  v_idx=1+(pose_index-1)*pose_dim;
endfunction;

function v_idx=landmarkMatrixIndex(landmark_index, num_poses, num_landmarks)
  global pose_dim;
  global landmark_dim;
  if (landmark_index>num_landmarks)
    printf("error matrix index: land idx > num_landmark\n")
    v_idx=-1;
    return;
  endif;
  v_idx=1 + (num_poses)*pose_dim + (landmark_index-1) * landmark_dim;
endfunction;

#boxplus operator to add perturbation in manifold space
function [XR, XL]=boxPlus(XR, XL, num_poses, num_landmarks, dx)
  global pose_dim;
  global landmark_dim;
  for(pose_index=1:num_poses)
    pose_matrix_index=poseMatrixIndex(pose_index, num_poses, num_landmarks);
    dxr=dx(pose_matrix_index:pose_matrix_index+pose_dim-1);
    #for poses we can't do simple addition, here is why boxplus is needed
    XR(:,:,pose_index)=v2t(dxr)*XR(:,:,pose_index);
    
  endfor;
  for(landmark_index=1:num_landmarks)
    #this is needed because some landmarks were not considered
    if landmark_index<=length(XL) && XL(landmark_index).pos
      landmark_matrix_index=landmarkMatrixIndex(landmark_index, num_poses, num_landmarks);
      dxl=dx(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,:);
      #nothing special needed for landmark
      XL(landmark_index).pos+=dxl;
    endif
  endfor;
endfunction;


function [e, Jr, Jl] = errorAndJacobian(Xr, Xl, z)
    R = Xr(1:2,1:2);
    t = Xr(1:2,3);

    z_hat = norm( R' * (Xl-t));
    #error
    #estimated measurement - actual measurement
    e = z_hat - z;
    
    #computing jacobian
    p=R' * (Xl-t);
    c=Xr(1,1); s=Xr(2,1);
    x=Xr(1,3); y=Xr(2,3);
    xl=Xl(1);  yl=Xl(2);
    
    #derivative of error with respect to state
    f1=(c*x-c*xl+s*y-s*yl);  #recurrent term
    f2=(-s*x+s*xl+c*y-c*yl); #other recurrent term
    #Jr(3) is 0, error do not depends from angle!
    #could have done with syms variables, but with this is faster and I forgot to use it before computing everything by hand
  
    Jr=(1/norm(p)) * [f1*c+f2*-s, f1*s+f2*c, f1*( (-x*s)+(xl*s)+(y*c)+(-yl*c)) + f2*( (-x*c)+(xl*c)+(-y*s)+(yl*s)) ];
    Jl=(1/norm(p)) * [f1*-c+f2*s f1*-s+f2*-c];
    

endfunction
