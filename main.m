close all
clear 
clc


addpath 'tools/g2o_wrapper'
addpath 'tools/visualization'
addpath 'tools/my_visualization'
addpath 'tools/algorithms'
source 'tools/algorithms/initGuess.m'
source 'tools/algorithms/leastSquares.m'
source 'tools/algorithms/LeastSquaresUtils.m'
source 'tools/algorithms/evaluate.m'
source 'tools/algorithms/tuning.m'
source 'tools/my_visualization/myPlot.m'

%get data
data_path="data";
dataset_gt="slam2d_range_only_ground_truth.g2o";
dataset_ig="slam2d_range_only_initial_guess.g2o";

printf("\n\nLoading Data...\n")
[gt_landmarks, gt_poses, gt_transitions, gt_observations] = loadG2o(fullfile(data_path, dataset_gt));
[~,poses,transitions,observations] = loadG2o(fullfile(data_path, dataset_ig));

%cardinality of the data
num_landmarks=length(gt_landmarks);
num_poses=length(poses);
num_transitions=length(transitions);
num_observations=length(observations);
printf("number of landmarks: %d\nnumber of poses: %d\nnumber of measurements: %d\n",num_landmarks,num_poses,num_observations)


%Set some parameters
first_pose_id=poses(1).id;
params.first_pose_id=first_pose_id;

params.kernel_threshold=2.0;
params.num_iterations=25;
params.damping=0.1;
printf("parameters:\n")
params

%----INITIAL GUESS----
printf("\n Initial Guess...\n")
[landmark_positions,robot_poses,land_observations,land_id_to_idx]=initGuess(observations,poses,transitions,params);
init_guess=land_observations;
printf("Error in landmark position using initial guess is %f\n",evaluate(gt_landmarks,land_observations,land_id_to_idx));


%---LEAST SQUARE OPTIMIZATION-------
printf("\n Least squares using %d iterations...\n",params.num_iterations)
[XR,XL]=leastSquares(land_observations,robot_poses,landmark_positions,poses,gt_landmarks,params,land_id_to_idx);

printf("error in landmark position after least square is %f\n",evaluate(gt_landmarks,XL,land_id_to_idx));

%tuning parameters
%this function was used to test if changing parameters of LS (kernel_threshold, damping) had some effect
%[XR,XL]=tuning(land_observations,robot_poses,landmark_positions,poses,gt_landmarks,params,land_id_to_idx);


%----PLotting----
printf("\nPlotting...\n")

%to plot estimated landmarks and ground truth with labels
%this functions however don't use traditional octave plot so the legend of the other plot will be wrong
%drawLandmarkWithLabel(XL,gt_landmarks,radius=0.1)

%get ground truth poses in matrix form, so it's easier to plot and legend work
gt_matrix_poses=zeros(3, 3, length(gt_poses));
for i=1:length(gt_poses)
    gt_matrix_poses(:,:,i)=v2t([gt_poses(i).x gt_poses(i).y gt_poses(i).theta]);
endfor

%plot trajectory and landmarks
myPlot(robot_poses,XR,gt_matrix_poses,init_guess,XL,gt_landmarks)

printf("Done!\n")

