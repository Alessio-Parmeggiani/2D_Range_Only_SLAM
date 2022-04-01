1;
function T=preprocessTransitions(transitions,poses)

    for i=1:length(transitions)

        pose_prev=poseFromId(transitions(i).id_from,poses);
        pose_next=poseFromId(transitions(i).id_to,poses);
        T(end+1).pose_prev=v2t([pose_prev.x pose_prev.y pose_prev.theta]');
        T(end).pose_new=v2t([pose_next.x pose_next.y pose_next.theta]');
        T(end).v=transitions(i).v;
    endfor

endfunction