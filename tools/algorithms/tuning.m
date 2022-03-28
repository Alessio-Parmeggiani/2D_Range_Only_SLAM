1;
function kernel_threshold=tuning(land_observations,robot_poses,landmark_positions,poses,gt_landmarks,params)
    %greedy parameter tuning
    %choose parameter p and increment 
    %if new parameter improve result use it and test p+increment
    %otherwise reduce increment and p=p-increment
    params.damping=0.05;
    best_err=100000;
    incr=0.5;
    iterations=3;
    for i=1:iterations
        printf("trying damp=%f\n",params.damping)
        [XR,XL]=leastSquares(land_observations,robot_poses,landmark_positions,poses,gt_landmarks,params);
        err=evaluate(gt_landmarks,XL)
        if err<best_err
            best_err=err;
            params.damping+=incr;
        else
            params.damping-=incr;
            incr=incr/2;
            params.damping-=incr;
        endif
    endfor
    printf("best damp is%f\n",params.damping)
endfunction