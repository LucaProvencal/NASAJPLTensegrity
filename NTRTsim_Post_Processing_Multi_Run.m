%% NTRTsim Post-Processing - NASA JPL Tensegrity - ME4702 Capstone Design
%
%  author: Luca Provencal
%
%  Performs the post-processing on NTRTsim data logs.
%  ***********************************************************************
close all
clc
clear
%  ***********************************************************************

folderpath = '/Users/LucaProvencal/Documents/NEU/2020_Spring/ME4702/Shared_Tensegribuntu/adskjhfa/';

filelist   = dir(folderpath);
name       = {filelist.name};
date       = {filelist.date};
name       = name(~strncmp(name, '.', 1));
date       = date(~strncmp(date, '.', 1));

finalArr = [];
for i=1:length(name)
    file = string(name(1, i));

    filename = strcat(folderpath, file);

    data = readtable(filename);

    splitTitle = split(file{1}(1:end), "_");
    stiffnessouter = str2num(splitTitle{2});
    stiffnessinner = str2num(splitTitle{3});
    pretensionouter = str2num(splitTitle{4});
    pretensioninner = str2num(splitTitle{5});

    % take derivative twice for position and accel vs time
    dydx = gradient(data{:,3}./10)./gradient(data{:,1});
    d2ydx2 = gradient(dydx(:))./gradient(data{:,1});

    % Update xlim and ylim accordingly in each of the subplots
    subplot(3,1,1)
    plot(data{:,1}, data{:,3}./10)
    %     xlim([.1 .195])
    xlabel('Time (s)')
    ylabel('Position (m)')
    title('Position vs. Time') % Center payload at impact

    subplot(3,1,2)
    plot(data{:,1}, dydx(:))
%     xlim([.1 .195])
    % %     xlabel('Time (s)')
    ylabel('Velocity (m/s)')
    title('Velocity vs. Time')

    subplot(3,1,3)
    plot(data{:,1}, d2ydx2(:))
    %     xlim([.1 .195])
    xlabel('Time (s)')
    ylabel('Acceleration (m/s^2)')
    title('Accel vs. Time')
    xlim([1 1.035])
    ylim([-400 2000])

    % sum of times that the payload nearly hits the ground. .0331 is a
    % threshold that can be adjusted.
    hits_ground = sum(data{:,3}./10 < .0331);

    maxAccel = max(d2ydx2);

    resultsArr = [stiffnessouter, stiffnessinner, pretensionouter, pretensioninner, maxAccel, hits_ground];

    finalArr = [finalArr; resultsArr];

end

% results stored in finalArr table
finalArr = sortrows(finalArr);
finalTable = array2table(finalArr);


%% get compressed force
%  first find compressed distance

% index 45 for no inner members
% index 52 for inner members

% min_compression = (min(data{:,45}) - .25) *  3.937; % dm to inches
% min_compression_meters = (min(data{:,45}) - .25) *  .1; % dm to m
% radius = 2; % dm
% density = .40; % kg/dm^3
% length = .5;
%
% vol = pi() * radius^2 * length;
% mass = vol*density;
%
% force = 9.81 * mass;
