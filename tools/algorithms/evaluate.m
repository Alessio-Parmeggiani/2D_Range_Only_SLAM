1;
function err=evaluate(gt_landmarks, est_landmarks,land_id_to_idx)
    % error is computed as simply the distance between the correct 
    % landmark and the estimated position
    err=0;
    for i=1:length(gt_landmarks)
        %for each landmark retrieving original index and compute error
        gt_id=gt_landmarks(i).id;
        est_idx=land_id_to_idx(gt_id);
        if gt_id==est_landmarks(est_idx).l_id
            gt_pos=[ gt_landmarks(i).x_pose;  gt_landmarks(i).y_pose];
            est_pos=est_landmarks(est_idx).pos;
            if est_pos
                err+=norm(gt_pos -est_pos);
            endif
        else 
            %should not happen
            printf("ERROR! indices do not correspond")
            printf("  %d %d\n",gt_id,est_landmarks(est_idx).l_id)
        endif
    endfor
    %computing final error
    err=err/length(est_landmarks);
endfunction


function err=evaluatePoses(gt_poses,est_poses)
    err=0;
    for i=1:length(gt_poses)
        p1=t2v(gt_poses(:,:,i));
        p2=t2v(est_poses(:,:,i));
        err+=norm(p1-p2);
    endfor
    err=err/length(gt_poses);
endfunction