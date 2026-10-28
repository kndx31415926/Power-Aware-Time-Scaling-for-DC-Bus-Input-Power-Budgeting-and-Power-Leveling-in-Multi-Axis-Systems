function Trajectory=spinn3d_read_alpha_trajectory(folder,~,~,load_data)
Trajectory=[];
if load_data
    S=load(fullfile(folder,'trajectory.mat'),'Trajectory');
    Trajectory=S.Trajectory;
end
end
