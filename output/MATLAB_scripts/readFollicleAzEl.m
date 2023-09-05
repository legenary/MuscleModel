function [az, el] = readFollicleAzEl(path, horizontal)

    [top, bot] = readFollicleTopBot(path);
    
    %% Transform fol_trajectory to specified plane in two steps
    if strcmp(horizontal, 'averagerowplane') || strcmp(horizontal, 'eyenose')
        % First, adjust to average row plane
        R = [0.9092,0.3570,0.2142;
            -0.3598,0.9326,-0.0270;
            -0.2094,-0.0525,0.9764];
        top_bot_avgrow = cellfun(@(x) x*R, [top, bot], 'uni', 0);
        top_bot = top_bot_avgrow;

        if strcmp(horizontal, 'eyenose')
            % Then, adjust to eye-nose plane (Knutsen et al 2008) from avg row plane
            % calculate eye-nose plane
            eyes = [-9.721020847417353,-19.067448129825216,3.891614295391962;
                -6.268610691514697,-13.559304383739748,5.840277177970524;
                10.292905691029162,-19.444559678246000,3.425704247361483;
                7.208632295799561,-14.387253506531858,5.566668713822409];
            noses = [-1.928577625443136,5.716489592791897,1.605123645981586;
                -0.734239178172683,7.323191087283126,0.977908851272219;
                2.622476944320339,5.453629555452386,1.748427308939650;
                1.341145614798153,7.520348466123311,1.065171840940149];
            eye2nose = mean(noses) - mean(eyes);

            % angle is from average row plane to eye-nose plane:
            angle = atand(eye2nose(3)/eye2nose(2));
            top_bot_eyenose = cellfun(@(x) x*rotx(angle, 'deg'), top_bot_avgrow, 'uni', 0);
            top_bot = top_bot_eyenose;
        end
    else
        error("specify which horizontal plane? averagerowplane or eyenose");
        
    end
    
    stepTotal = size(top{1, 1}, 1);
    nFol = size(top, 1);
    az = zeros(stepTotal, nFol);
    el = zeros(stepTotal, nFol);
    for i = 1:nFol
        dir = top_bot{i, 1} - top_bot{i, 2};
        [th, phi, ~] = cart2sph(dir(:,1), dir(:,2), dir(:,3));
        az(:, i) = rad2deg(th) + 90;
        el(:, i) = rad2deg(phi);
    end


end