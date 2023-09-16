
% at the end of this script, vec_top2D means the basepoint location, which is the top end of the follicle,
% and vec_bot2D means the deeper end of the follicle.

% this script generates follicle information based on data of NewRatMap

clear; close all;

load('NewRatMapModelOutput.mat');
PadResliced = load('E:\Northwestern University\Hartmann Lab - Mitra Hartmann - __YifuLuo_activeWorking\_ARP\04c_Reslice\data_resliced\2019-17LReslice1.mat').FolResliced;

PadResliced(6, :) = {'a0','a1','a2','a3','a4','a5','a6','a7',...
    'b0','b1','b2','b3','b4','b5','b6','b7',...
    'c0','c1','c2','c3','c4','c5','c6','c7',...
    'd0','d1','d2','d3','d4','d5','d6','d7',...
    'e0','e1','e2','e3','e4','e5','e6','e7',...
    };

% but only part of them are valid
PadResliced(:, [6, 7, 8, 14, 15, 16, 23, 24, 33]) = [];
% PadResliced()

%% get follicle top from new rat map
% sort two ends so that they're top and bottom
% figure; hold on
R = [0.9092,0.3570,0.2142;
     -0.3598,0.9326,-0.0270;
     -0.2094,-0.0525,0.9764];
follicle_pos_ypr_len_vol = zeros(31, 8);
for i = 1:31
    name = PadResliced{6, i};
    row = name(1)-'a'+1;
    col = name(2)-'0'+1;
    idx = find(modelRowW==row & modelColW==col);
    if ~isempty(idx)
        dir = EulerRotateWhiskerYifu(modelThetaW(idx), modelPhiW(idx), modelZetaW(idx),[1,0]);
        vec_length = PadResliced{2,i};
        %%% option 1 use rat map basepoint location
%         vec_top = modelPointsBP(idx, :);
%         vec_bot = vec_top - vec_length*dir';
%         
%         vec_top_ARP = vec_top * R';
%         vec_top_ARP(3) = -vec_top_ARP(3);
%         vec_top_ARP = vec_top_ARP * roty(-pi/2)' * rotz(pi/2)';
% 
%         vec_bot_ARP = vec_bot * R';
%         vec_bot_ARP(3) = -vec_bot_ARP(3);
%         vec_bot_ARP = vec_bot_ARP * roty(-pi/2)' * rotz(pi/2)';
        
        %%% option 2 use ARP basepoint location
        if PadResliced{4,i}(3) > PadResliced{5,i}(3)
            vec_top_ARP = PadResliced{4,i}/1000;
        else
            vec_top_ARP = PadResliced{5,i}/1000;
        end
        vec_top_ARP(2) = -vec_top_ARP(2);
        
        dir_ARP = (vec_length*dir') * R';
        dir_ARP(3) = -dir_ARP(3);
        dir_ARP = dir_ARP * roty(-pi/2)' * rotz(pi/2)';
        
        vec_bot_ARP = vec_top_ARP - dir_ARP;
        
        
%         plot3d([vec_top_ARP; vec_bot_ARP], 'r-');
        
        follicle_pos_ypr_len_vol(i, 1:3) = (vec_top_ARP + vec_bot_ARP) /2 ;
        direction_vector = vec_top_ARP - vec_bot_ARP;
        [th, phi, r] = cart2sph(direction_vector(1), direction_vector(2), direction_vector(3));
        follicle_pos_ypr_len_vol(i, 4:6) = [th, phi, 0];
        follicle_pos_ypr_len_vol(i, 7) = vec_length;
        follicle_pos_ypr_len_vol(i, 8) = PadResliced{3,i};
    end
end
% axis equal


% %%% compare to ARP data
% vec_top2D = cell2mat(cellfun(@(x) x', PadResliced(4,:), 'uni', 0))';
% vec_bot2D = cell2mat(cellfun(@(x) x', PadResliced(5,:), 'uni', 0))';
% % sort two ends so that they're top and bottom
% for i = 1:31
%     if vec_top2D(i, 3) < vec_bot2D(i, 3)
%         temp = vec_top2D(i, :);
%         vec_top2D(i, :) = vec_bot2D(i, :);
%         vec_bot2D(i, :) = temp;
%     end
% end
% % reverse y-axis
% vec_top2D(:, 2) = -vec_top2D(:, 2);
% vec_bot2D(:, 2) = -vec_bot2D(:, 2);
% % change unit to mm:
% vec_top2D = vec_top2D/1000;
% vec_bot2D = vec_bot2D/1000;
% for i = 1:31
%     plot3d([vec_top2D(i,:);vec_bot2D(i,:)], 'k-');
% end



writematrix(follicle_pos_ypr_len_vol, '../../follicle_pos_ypr_len_vol_fromRatMap.csv')
% save('follicle_pos.mat', 'vec_top2D', 'vec_bot2D');








