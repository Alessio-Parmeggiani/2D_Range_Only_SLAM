function pose=poseFromId(id,poses)
    #cost O(1)
    #this work because each pose has id = 1 + previous index 
    first_id=poses(1).id;
    pose=poses(id-first_id+1);

    #{
    #real loop used to check for real if they correspond, now not needed
    #it will be needed if the pose has not id=previous_id+1
    for i=1:length(poses)
        if poses(i).id==id
            pose=poses(i);
        endif
    endfor
    #}
endfunction
