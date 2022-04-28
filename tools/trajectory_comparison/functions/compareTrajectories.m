function stats = compareTrajectories(config)

% load the stuff

global cache

for i = 1:length(config.trjs)
    
    fprintf('Reading trajectory %d ...\n', i);
    
    file = dir(config.trjs{i}.path);
    if isempty(file)        
        error(sprintf('file %s does not exist\n', config.trjs{i}.path));
    end
    
    in_cache = false;
    
    cur_date = file.datenum;
    
    for j = 1:length(cache)
       if isequaln(config.trjs{i}, cache{j}.cfg)           
          if cache{j}.date == cur_date
              trjs{i} = cache{j}.trj;
              
              fprintf('Found in cache\n');
              in_cache = true;
              break;
          end              
       end
    end
    
    if in_cache ~= true || (isfield(config.trjs{i}, 'reload') && config.trjs{i}.reload == true)
        switch config.trjs{i}.type
            case 'roamfree'
                trjs{i} = loadTrajectory_roamfree(config.trjs{i});
                
            case 'posproc'
                trjs{i} = loadTrajectory_posproc(config.trjs{i});
        end
        
        cache{end+1} = struct('cfg', config.trjs{i}, 'date', cur_date, 'trj', trjs{i});
        
        fprintf('Done\n');
    end
end

% plot

close all

rpy_titles = {'Roll', 'Pitch', 'Yaw'};
enu_titles = {'East', 'North', 'Up'};

if isfield(config, 'diffwrtref') && config.diffwrtref == true
    i = setdiff(1:length(trjs),[config.reference]);
    
    if ~isfield(config, 't0')
        config.t0 = -inf;
    end
    if ~isfield(config, 'tend')
        config.tend = inf;
    end
    
    
%     idx_comp = find(trjs{config.reference}.t > config.t0-1e-6 & trjs{config.reference}.t < config.tend+1e-6);
%     t_comp = trjs{config.reference}.t(idx_comp);
    
    colors = get(gca,'colororder');
    
    %Plot Roll, Pitch, Yaw errors w.r.t reference trajectory
    for j = i
        
        idx_comp = find(trjs{j}.t > config.t0-1e-6 & trjs{j}.t < config.tend+1e-6);
        t_comp = trjs{j}.t(idx_comp);
        
        %rpy_int = interp1(trjs{j}.t, trjs{j}.rpy, trjs{config.reference}.t); 
        
        q_int = interp1(trjs{config.reference}.t, trjs{config.reference}.q, t_comp);
        
        q_diff = zeros(size(q_int,1), 4);
        for h = 1:size(q_int,1)
           % q_diff(h,:) = quatprod(quatinv(quatnorm(q_int(h,:))), trjs{config.reference}.q(idx_comp(h),:)); 
           q_diff(h,:) = quatprod(quatinv(trjs{j}.q(idx_comp(h),:)), quatnorm(q_int(h,:))); 
        end
        err_rpy = quat2euler(q_diff);
        
        for ax = 1:3
            
            err_rpy(:,ax) = rad2deg(unwrap(err_rpy(:, ax)));
            
            figure(1)
            subplot(3,1,ax)                       
            %plot(rad2deg( angdiff(trjs{config.reference}.rpy(:,ax), rpy_int(:,ax)) ));
            hold on
            
            plot(t_comp, err_rpy(:,ax), 'color', colors(find(i == j),:), 'DisplayName', config.trjs{j}.name);
            
            % linear interpolation
            coeffs = polyfit(t_comp, err_rpy(:,ax),1);
            plot(t_comp([1 end]), coeffs(1)*t_comp([1 end])+coeffs(2),'--','color', colors(find(i == j),:), 'lineWidth', 2, 'DisplayName', config.trjs{j}.name);
            
            if ax == 3                
                legend('Location','NorthEast')                
            end            
            grid on
            axis tight
            
            if (ax == 1 || ax == 2)            
                ylim(0.2*[-1 1])
            else
                ylim(0.4*[-1 1])
            end
            
            title(rpy_titles{ax})
            ylabel('[deg]');
            xlabel('time [s]');
            
            figure(2)           
            subplot(1,3,ax)
            hold on
            
            histogram(abs(err_rpy(:,ax)),'BinWidth',config.orient_binwidth(ax),'DisplayName', config.trjs{j}.name ,'Normalization','probability','EdgeAlpha',.2);       
            
            if ax == 3                
                legend('Location','NorthEast')                
            end            
            grid on
            axis tight
            
            if (ax == 1 || ax == 2)            
                xlim([0, 0.2])
            else
                xlim([0, 0.5])
            end
            
            title(rpy_titles{ax});
            ylabel('Probability [-]');
            xlabel('Error magnitude [°]');
            

        end
        
        stats{j}.MEAN_RPY = mean(err_rpy,1);
        stats{j}.RMSE_RPY = sqrt(mean(err_rpy.^2,1));
        stats{j}.STD_RPY  = std(err_rpy);
    end
    
    %Plot East North Up errors w.r.t reference trajectory and error hist
    for j = i
        idx_comp = find(trjs{j}.t > config.t0-1e-6 & trjs{j}.t < config.tend+1e-6);
        t_comp = trjs{j}.t(idx_comp);
        
%         enu_int = interp1(trjs{j}.t, trjs{j}.enu, t_comp);
        enu_int = interp1(trjs{config.reference}.t, trjs{config.reference}.enu, t_comp);
        
%         err_enu = trjs{config.reference}.enu(idx_comp,:) - enu_int;
        err_enu = enu_int - trjs{j}.enu(idx_comp,:);
        err_norm = sqrt(sum(err_enu.^2,2));
        
        for ax = 1:3
            
            figure(3)
            subplot(3,1,ax)            
            hold on
            plot(t_comp, err_enu(:,ax), 'DisplayName', config.trjs{j}.name, 'LineWidth', 0.75);
            
            if ax == 3                
                legend('Location','NorthEast')                
                
                xlabel('time [s]');
            end             
            grid on
            axis tight            
            title(enu_titles{ax})
            ylabel('[m]');
            
            
            figure(4)
            subplot(1,3,ax)            
            hold on
            histogram(err_enu(:,ax),'BinWidth',config.pos_binwidth(ax),'Normalization','probability','DisplayName', config.trjs{j}.name,'EdgeAlpha',.2)
            if ax == 3                
                legend('Location','NorthEast')                
            end             
            grid on
            axis tight
            title(enu_titles{ax})
            ylabel('Probability [-]');
            xlabel('Error [m]');
        end
        
        figure(5)
        legend
        %subplot(1,length(i),j-1)
        
        hold on
        
        histogram(err_norm,'BinWidth',.002,'Normalization','probability','DisplayName', config.trjs{j}.name,'EdgeAlpha',.2)
        title('Error magnitude distribution')
        ylabel('Probability [-]');
        xlabel('Error l2-norm [m]');
        
        stats{j}.MEAN_ENU = mean(err_enu,1);
        stats{j}.RMSE_ENU = sqrt(mean(err_enu.^2,1));
        stats{j}.STD_ENU  = std(err_enu);
        
        
        stats{j}.Err_MEAN = mean(err_norm);
        stats{j}.Err_MIN = min(err_norm);
        stats{j}.Err_MAX = max(err_norm);
        stats{j}.Err_STD = std(err_norm);
    end
    
else
    figure
    for ax = 1:3
        subplot(1,3,ax)
        hold on

        for i = 1:length(trjs)
            plot(trjs{i}.t, rad2deg(trjs{i}.rpy(:,ax)))
        end    

        grid on
        axis tight
        
        title(rpy_titles{ax});
        ylabel('[deg]');
        xlabel('time [s]');
    end
    
    figure
    for ax = 1:3
        subplot(1,3,ax)
        hold on

        for i = 1:length(trjs)
            plot(trjs{i}.t, trjs{i}.enu(:,ax))
        end    

        grid on
        axis tight
        
        title(enu_titles{ax});
        ylabel('[m]');
        xlabel('time [s]');
    end
   
end

end

function [t, enu, q] = load_roamfree_raw(cfg)
    if strcmp(cfg.path(end-5:end),'tar.gz')

        j = find(cfg.path == '/', 1, 'last');
        tmp_path = ['/tmp/' cfg.path(j+1:end-7)];

        if exist(tmp_path, 'file') == 7  
            rmdir(tmp_path, 's');
        end

        mkdir(tmp_path);
        system(['tar xzf ''' cfg.path ''' -C ''' tmp_path '''']);

        cfg.path = tmp_path;    
    end

    raw = load([cfg.path '/PoseSE3(W).log']);
    
    [~, j] = sort(raw(:,1));

    t = raw(j,1);
    enu = raw(j,3:5);
    q = raw(j,6:9);
end    

function trj = loadTrajectory_roamfree(cfg)
    [trj.t, trj.enu, trj.q] = load_roamfree_raw(cfg);
    
    if isfield(cfg, 'Qbs')
        for i = 1:length(trj.t)
            trj.q(i,:) = quatprod(trj.q(i,:), cfg.Qbs);
        end
    end
    
    trj.rpy = quat2euler(trj.q);
end

function trj = loadTrajectory_posproc(cfg)

    psp = readPosprocFile(cfg.path, 'Format', 'SNV')';
    
    trj.t = psp(:,1);
    
    if isfield(cfg, 'timeOffset')
        trj.t = trj.t + cfg.timeOffset;
    end    
    
    rpy_orig = psp(:,8:10);
    
    % construct rotation matrix
    
    R_enu_ned = [0 1 0; 1 0 0; 0 0 -1]; %the same as T_ned_enu
    
    R_e_l0 = R_e_l_enu(deg2rad(cfg.lat0), deg2rad(cfg.lon0));
    
    trj.q = zeros(length(trj.t), 4);    
    for i = 1:length(trj.t)
        R_b_ned = (R1(rpy_orig(i,1)) * R2(rpy_orig(i,2)) * R3(rpy_orig(i,3)));
        R_enu_b = R_enu_ned*R_b_ned';
       
        trj.q(i,:) = dcm2quat(R_enu_b);
    end
    
    ecef = lla2ecef([rad2deg(psp(:,2:3)) psp(:,4)], 'WGS84');    
    [E,N,U] = ecef2enu(ecef(:,1), ecef(:,2), ecef(:,3), cfg.lat0, cfg.lon0, 0, wgs84Ellipsoid('meter'));  
    
    trj.rpy = quat2euler(trj.q);
    trj.enu = [E N U];

end

function R_e_l = R_e_l_enu(lat_rad, lon_rad)
R_e_l = [ -sin(lon_rad)       -sin(lat_rad)*cos(lon_rad)       cos(lat_rad)*cos(lon_rad);
           cos(lon_rad)       -sin(lat_rad)*sin(lon_rad)       cos(lat_rad)*sin(lon_rad);
                      0                     cos(lat_rad)                    sin(lat_rad)];
end